# MicroPython USB CDC module
# MIT license; Copyright (c) 2022 Martin Fischer, 2023 Angus Gratton
import io
import ustruct
import time
import errno
from micropython import const

from .device import (
    USBInterface,
    get_usbdevice
)
from .utils import (
    Buffer,
    endpoint_descriptor,
    split_bmRequestType,
    STAGE_SETUP,
    REQ_TYPE_STANDARD,
    REQ_TYPE_CLASS,
    EP_IN_FLAG
)

_DEV_CLASS_MISC = const(0xef)
_CS_DESC_TYPE = const(0x24)   # CS Interface type communication descriptor
_ITF_ASSOCIATION_DESC_TYPE = const(0xb)  # Interface Association descriptor

# CDC control interface definitions
_INTERFACE_CLASS_CDC = const(2)
_INTERFACE_SUBCLASS_CDC = const(2)  # Abstract Control Mode
_PROTOCOL_NONE = const(0)   # no protocol

# CDC descriptor subtype
# see also CDC120.pdf, table 13
_CDC_FUNC_DESC_HEADER = const(0)
_CDC_FUNC_DESC_CALL_MANAGEMENT = const(1)
_CDC_FUNC_DESC_ABSTRACT_CONTROL = const(2)
_CDC_FUNC_DESC_UNION = const(6)

# Other definitions
_CDC_VERSION = const(0x0120)  # release number in binary-coded decimal


# CDC data interface definitions
_CDC_ITF_DATA_CLASS = const(0xa)
_CDC_ITF_DATA_SUBCLASS = const(0)
_CDC_ITF_DATA_PROT = const(0)   # no protocol

# Length of the bulk transfer endpoints. Maybe should be configurable?
_BULK_EP_LEN = const(64)

class CDC(io.IOBase):
    # USB CDC serial device class, designed to resemble machine.UART
    # with some additional methods.
    #
    # This is a standalone class, instead of a USBInterface subclass, because
    # CDC consists of multiple interfaces (CDC control and CDC data)
    def __init__(self):
        # For CDC to work, the device class must be set to Interface Association
        usb_device = get_usbdevice()
        usb_device.device_class = _DEV_CLASS_MISC
        usb_device.device_subclass = 2
        usb_device.device_protocol = 1   # Itf association descriptor

        self._ctrl = CDCControlInterface()
        self._data = CDCDataInterface()
        # The data interface *must* be added immediately after the control interface
        usb_device.add_interface(self._ctrl)
        usb_device.add_interface(self._data)

        # TODO: Add kwargs and call init() with kwargs

    def init(self, baudrate=9600, bits=8, parity=None, stop=1, timeout=None, txbuf=0, rxbuf=0, flow=0):
        # Configure the CDC serial port. Note that many of these settings like
        # baudrate, bits, parity, stop don't change the USB-CDC device behavior
        # at all, only the "line coding" reported to the USB host.

        # TODO: Handle baudrate, bits, parity, stop

        if flow != 0:
            raise NotImplementedError  # TODO: flow control not supported

        self._data._timeout = timeout
        self._data._wb = Buffer(txbuf) if txbuf else None
        self._data._rb = Buffer(rxbuf) if rxbuf else None

    def is_open(self):
        return self._ctrl.is_open()

    ###
    ### io.IOBase stream implementation
    ###

    def read(self, size=-1):
        return self._data.read(size)

    def readinto(self, b):
        return self._data.readinto(size)

    def write(self, buf):
        return self._data.write(buf)

    def ioctl(self, req, arg):
        raise NotImplementedError  # TODO




class CDCControlInterface(USBInterface):
    # Implements the CDC Control Interface

    def __init__(self):
        super().__init__(_INTERFACE_CLASS_CDC, _INTERFACE_SUBCLASS_CDC, _PROTOCOL_NONE)

    def get_itf_descriptor(self, num_eps, itf_idx, str_idx):
        # CDC needs a Interface Association Descriptor (IAD)
        # two interfaces in total
        desc = ustruct.pack("<BBBBBBBB",
                            8,
                            _ITF_ASSOCIATION_DESC_TYPE,
                            itf_idx,
                            2,
                            _INTERFACE_CLASS_CDC,
                            _INTERFACE_SUBCLASS_CDC,
                            _PROTOCOL_NONE,
                            0)

        itf, strs = super().get_itf_descriptor(num_eps, itf_idx, str_idx)
        desc += itf
        # Append the CDC class-specific interface descriptor
        # see CDC120-track, p20
        desc += ustruct.pack("<BBBH",
                             5,  # bFunctionLength
                             _CS_DESC_TYPE,  # bDescriptorType
                             _CDC_FUNC_DESC_HEADER,  # bDescriptorSubtype
                             _CDC_VERSION)  # cdc version

        # CDC-PSTN table3 "Call Management"
        # set to No
        desc += ustruct.pack("<BBBBB",
                             5,  # bFunctionLength
                             _CS_DESC_TYPE,  # bDescriptorType
                             _CDC_FUNC_DESC_CALL_MANAGEMENT,  # bDescriptorSubtype
                             0,  # bmCapabilities - XXX no call managment so far
                             1)  # bDataInterface - interface 1

        # CDC-PSTN table4 "Abstract Control"
        # set to support line_coding and send_break
        desc += ustruct.pack("<BBBB", 
                             4,  # bFunctionLength
                             _CS_DESC_TYPE,  # bDescriptorType
                             _CDC_FUNC_DESC_ABSTRACT_CONTROL,  # bDescriptorSubtype
                             0x6)  # bmCapabilities D1, D2 
        # CDC-PSTN "Union"
        # set control interface / data interface number
        desc += ustruct.pack("<BBBBB",
                             5,  # bFunctionLength
                             _CS_DESC_TYPE,  # bDescriptorType
                             _CDC_FUNC_DESC_UNION,  # bDescriptorSubtype
                             itf_idx,  # bControlInterface
                             itf_idx+1)  # bSubordinateInterface0 (data class itf number)
        return desc, strs

    def get_endpoint_descriptors(self, ep_addr, str_idx):
        self.ep_in = ep_addr | EP_IN_FLAG
        desc = endpoint_descriptor(self.ep_in, "interrupt", 8, 16)
        return (desc, [], (self.ep_in,))

    def handle_interface_control_xfer(self, stage, request):
        # Handle standard and class-specific interface control transfers for CDC devices.
        bmRequestType, bRequest, wValue, _, _ = request
        recipient, req_type, _ = split_bmRequestType(bmRequestType)

        print(f'itf cntrl: {recipient}, {req_type}')
        super().handle_interface_control_xfer(stage, request)


class CDCDataInterface(USBInterface):
    # Implements the CDC Data Interface

    def __init__(self):
        super().__init__(_CDC_ITF_DATA_CLASS, _CDC_ITF_DATA_SUBCLASS,
                         _CDC_ITF_DATA_PROT)
        self._wb = ()  # Optional write Buffer (IN endpoint), set by CDC.init()
        self._rb = ()  # Optional read Buffer (OUT endpoint), set by CDC.init()
        self._timeout = 0  # set from CDC.init() as well

    def get_endpoint_descriptors(self, ep_addr, str_idx):
        self.ep_in = ep_addr | EP_IN_FLAG
        self.ep_out = ep_addr
        # one IN / OUT Endpoint
        e_out = endpoint_descriptor(self.ep_out, "bulk", _BULK_EP_LEN, 0)
        e_in = endpoint_descriptor(self.ep_in, "bulk", _BULK_EP_LEN, 0)
        return (e_out + e_in, [], (self.ep_out, self.ep_in))

    def write(self, buf):
        # use a memoryview to track how much of 'buf' we've written so far
        # (unfortunately, this means a 1 block allocation for each write, but it's otherwise allocation free.)
        mv = memoryview(buf)
        start = time.ticks_ms()
        while mv:
            if self._timeout and time.ticks_diff(time.ticks_ms(), start) > self._timeout:
                # TODO: in unbuffered mode, need to cancel the pending transfer
                raise OSError(errno.ETIMEDOUT)  # TODO: check if should do this, or return number of bytes written

            # TODO: check for failed USB transfers

            if self._wb:
                # Keep pushing buf into _wb into it's all gone
                nbytes = self._wb.write(mv)
                if nbytes:
                    mv = mv[nbytes:]
                self._wr_xfer()  # make sure a transfer is running from _wb
            else:
                # Transmit 'buf' synchronously.
                #
                # TODO: Currently this tries the whole write in one pass,
                # need to check if better to send _BULK_EP_LEN at a time from
                # here or if TinyUSB will fragment for us
                def cb(_e, _r, nbytes):
                    nonlocal mv
                    # TODO: handle error case
                    print("no_buf_cb", len(mv), nbytes)
                    mv = mv[nbytes:]
                if not self.xfer_pending(self.ep_in):
                    self.submit_xfer(self.ep_in, mv, cb)

            if mv:
                time.sleep_ms(10)

        # Note: if a tx buffer is set then this returns when all bytes are in the tx buffer,
        # not necessarily when the USB transfers have all completed
        return len(buf)


    def _wr_xfer(self):
        # Submit a new IN transfer from the _wb buffer
        if self._wb and self._wb.readable() and not self.xfer_pending(self.ep_in):
            print('_wr_xfer buffered')
            self.submit_xfer(self.ep_in,
                             self._wb.pend_read(),
                             self._wr_cb)

    def _wr_cb(self, ep, res, num_bytes):
        # Whenever a buffered IN transfer ends
        # TODO: check res
        print('_wr_cb', res, num_bytes)
        self._wb.finish_read(num_bytes)
        self._wr_xfer()

    def _rd_xfer(self):
        # Keep an active OUT transfer to read data from the host,
        # whenever there is a receive buffer with room for new data
        if self._rb and self._rb.writable() and not self.xfer_pending(self.ep_out):
            self.submit_xfer(self.ep_out, self._rb.pend_write(), self._rd_cb)

    def _rd_cb(self, ep, res, num_bytes):
        print('_rd_cb', res, num_bytes)
        # TODO: check res
        self._rb.finish_write(num_bytes)
        self._rd_xfer()

    def handle_open(self):
        super().handle_open()
        print("open")
        self._rd_xfer()

    def read(self, size):
        # TODO: Support non-blocking
        start = time.ticks_ms()

        # Allocate a suitable buffer to read into
        if size >= 0:
            b = bytearray(size)
        elif self._rb:
            # for size == -1 and read buffer, return however many bytes are ready
            b = bytearray(self._rb.readable())
        else:
            # No read buffer, so try to read up to the endpoint length
            b = bytearray(_BULK_EP_LEN)

        n = self._readinto(b, start, size)
        return b[:n]  # TODO: check if this allocates if n == len(b)


    def readinto(self, b):
        return self._readinto(b, time.ticks_ms(), len(b))

    def _readinto(self, b, start, size):
        # TODO: Support non-blocking

        n = 0
        try:
            while True:
                if self._timeout and time.ticks_diff(time.ticks_ms(), start) > self._timeout:
                    # Timed out
                    return n

                if not self._rb:
                    # no read buffer, so submit a transfer directly into 'res'
                    def cb(_e, _r, num_bytes):
                        nonlocal n
                        n += num_bytes
                    if not self.xfer_pending(self.ep_out):
                        self.submit_xfer(self.ep_out, memoryview(b)[n:], cb)
                else:
                    # there is a read buffer, so try and read out of it
                    n += self._rb.readinto(b[n:])

                if n and size == -1:
                    # For size == -1, return as soon as we have something
                    return n

                if n < len(b):
                    time.sleep_ms(10)

            return n
        finally:
            if not self._rb and self.xfer_pending(self.ep_out):
                pass # TODO: cancel any pending no-read-buffer transfer

