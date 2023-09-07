# MicroPython USB utility functions
# MIT license; Copyright (c) 2023 Angus Gratton
#
# Some constants and stateless utility functions for working with USB descriptors and requests.
from micropython import const
import machine
import ustruct

if not hasattr(machine, 'disable_irq'):
    # Allow testing on the unix port
    # TODO: Remove or make less hacky before merging
    class FakeMachine:
        def disable_irq(self):
            return -99
        def enable_irq(self, s):
            pass
    machine = FakeMachine()

# Shared constants
#
# It's a tough decision of when to make a constant "shared" like this. "Private" constants have no resource use, but these will take up flash space for the name. Suggest deciding on basis of:
#
# - Is this constant used in a lot of places, including potentially by users
#   of this package?
#
# Otherwise, it's not the greatest sin to be copy-pasting "private" constants
# in a couple of places. I guess. :/

EP_IN_FLAG = const(1 << 7)

# Control transfer stages
STAGE_IDLE = const(0)
STAGE_SETUP = const(1)
STAGE_DATA = const(2)
STAGE_ACK = const(3)

# Request types
REQ_TYPE_STANDARD = const(0x0)
REQ_TYPE_CLASS = const(0x1)
REQ_TYPE_VENDOR = const(0x2)
REQ_TYPE_RESERVED = const(0x3)

# TinyUSB xfer_result_t enum
RESULT_SUCCESS = const(0)
RESULT_FAILED = const(1)
RESULT_STALLED = const(2)
RESULT_TIMEOUT = const(3)
RESULT_INVALID = const(4)


# Non-shared constants, used in this function only
_STD_DESC_ENDPOINT_LEN = const(7)
_STD_DESC_ENDPOINT_TYPE = const(0x5)


def endpoint_descriptor(bEndpointAddress, bmAttributes, wMaxPacketSize, bInterval=1):
    # Utility function to generate a standard Endpoint descriptor bytes object, with
    # the properties specified in the parameter list.
    #
    # See USB 2.0 specification section 9.6.6 Endpoint p269
    #
    # As well as a numeric value, bmAttributes can be a string value to represent
    # common endpoint types: "control", "bulk", "interrupt".
    bmAttributes = {"control": 0, "bulk": 2, "interrupt": 3}.get(bmAttributes, bmAttributes)
    return ustruct.pack(
        "<BBBBHB",
        _STD_DESC_ENDPOINT_LEN,
        _STD_DESC_ENDPOINT_TYPE,
        bEndpointAddress,
        bmAttributes,
        wMaxPacketSize,
        bInterval,
    )


def split_bmRequestType(bmRequestType):
    # Utility function to split control transfer field bmRequestType into a tuple of 3 fields:
    #
    # Recipient
    # Type
    # Data transfer direction
    #
    # See USB 2.0 specification section 9.3 USB Device Requests and 9.3.1 bmRequestType, p248.
    return (
        bmRequestType & 0x1F,
        (bmRequestType >> 5) & 0x03,
        (bmRequestType >> 7) & 0x01,
    )


class Buffer:
    # An interrupt-safe producer/consumer buffer that wraps a bytearray object.
    #
    # Kind of like a ring buffer, but supports the idea of returning a
    # memoryview for either read or write of multiple bytes (suitable for
    # passing to a buffer function without needing to allocate another buffer to
    # read into.)
    #
    # Consumer can call pend_read() to get a memoryview to read from, and then
    # finish_read(n) when done to indicate it read 'n' bytes from the
    # memoryview. There is also a readinto() convenience function.
    #
    # Producer must call pend_write() to get a memorybuffer to write into, and
    # then finish_write(n) when done to indicate it wrote 'n' bytes into the
    # memoryview. There is also a normal write() convenience function.
    #
    # - Only one producer and one consumer is supported.
    #
    # - Calling pend_read() and pend_write() is effectively idempotent, they can be
    #   called more than once without a corresponding finish_x() call if necessary
    #   (provided only one thread does this, as per the previous point.)
    #
    # - Calling finish_write() and finish_read() is hard interrupt safe (does
    #   not allocate). pend_read() and pend_write() each allocate 1 block for
    #   the memoryview that is returned.
    #
    # The buffer contents are always laid out as:
    #
    # - Slice [:_n] = bytes of valid data waiting to read
    # - Slice [_n:_w] = unused space
    # - Slice [_w:] = bytes of pending write buffer waiting to be written
    #
    # This buffer should be fast when most reads and writes are balanced and use
    # the whole buffer.  When this doesn't happen, performance degrades to
    # approximate a Python-based single byte ringbuffer.
    #
    def __init__(self, length):
        self._b = bytearray(length)
        self._n = 0  # number of bytes in buffer read to read, starting at index 0
        self._w = length  # start index of a pending write into the buffer, if any. equals len(self._b) if no write is pending.

    def writable(self):
        # Number of writable bytes in the buffer. Assumes no pending write is outstanding.
        return len(self._b) - self._n

    def readable(self):
        # Number of readable bytes in the buffer. Assumes no pending read is outstanding.
        return self._n

    def pend_write(self):
        # Returns a memoryview that the producer can write bytes into.
        ist = machine.disable_irq()
        try:
            self._w = self._n
            return memoryview(self._b)[self._w:]
        finally:
            machine.enable_irq(ist)

    def finish_write(self, nbytes):
        # Called by the producer to indicate it wrote nbytes into the buffer.
        ist = machine.disable_irq()
        try:
            assert nbytes <= len(self._b) - self._w  # can't say we wrote more than was pended
            if self._n < self._w:
                # data was read while the write was happening, so shuffle the buffer back towards index 0
                # to avoid fragmentation
                self._b[self._n:self._n+nbytes] = memoryview(self._b)[self._w:self._w+nbytes]
                self._n += nbytes
                self._w += nbytes
            else:
                # no data was read while the write was happening, so the buffer is already in place
                assert self._n == self._w
                self._n += nbytes
            self._w = len(self._b)
        finally:
            machine.enable_irq(ist)

    def write(self, w):
        # Helper method for the producer to write into the buffer in one call
        pw = self.pend_write()
        to_w = min(len(w), len(pw))
        if to_w:
            pw[:to_w] = w[:to_w]
            self.finish_write(to_w)
        return to_w

    def pend_read(self):
        # Return a memoryview that the consumer can read bytes from
        return memoryview(self._b)[:self._n]

    def finish_read(self, nbytes):
        # Called by the consumer to indicate it read nbytes from the buffer.
        ist = machine.disable_irq()
        try:
            assert nbytes <= self._n  # can't say we read more than was available
            i = 0
            self._n -= nbytes
            while i < self._n:
                # consumer only read part of the buffer, so shuffle remaining
                # read data back towards index 0 to avoid fragmentation
                self._b[i] = self._b[i + nbytes]
                i += 1
        finally:
            machine.enable_irq(ist)

    def readinto(self, b):
        # Helper method for the consumer to read out of the buffer in one call
        pr = self.pend_read()
        to_r = min(len(pr), len(b))
        if to_r:
            b[:to_r] = pr[:to_r]
            self.finish_read(to_r)
        return to_r
