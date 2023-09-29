# MicroPython USB CDC module
# MIT license; Copyright (c) 2022 Martin Fischer, 2023 Angus Gratton
import io
import ustruct
import time
import errno
import machine
import struct
from micropython import const

from .device import USBInterface, get_usbdevice
from .utils import (
    Buffer,
    endpoint_descriptor,
    split_bmRequestType,
    STAGE_SETUP,
    STAGE_DATA,
    STAGE_ACK,
    REQ_TYPE_STANDARD,
    REQ_TYPE_CLASS,
    EP_IN_FLAG,
)

_DEV_CLASS_MISC = const(0xEF)
_CS_DESC_TYPE = const(0x24)  # CS Interface type communication descriptor
_ITF_ASSOCIATION_DESC_TYPE = const(0xB)  # Interface Association descriptor

# CDC control interface definitions
_INTERFACE_CLASS_CDC = const(2)
_INTERFACE_SUBCLASS_CDC = const(2)  # Abstract Control Mode
_PROTOCOL_NONE = const(0)  # no protocol

# CDC descriptor subtype
# see also CDC120.pdf, table 13
_CDC_FUNC_DESC_HEADER = const(0)
_CDC_FUNC_DESC_CALL_MANAGEMENT = const(1)
_CDC_FUNC_DESC_ABSTRACT_CONTROL = const(2)
_CDC_FUNC_DESC_UNION = const(6)

# CDC class requests, table 13, PSTN subclass
_SET_LINE_CODING_REQ = const(0x20)
_GET_LINE_CODING_REQ = const(0x21)
_SET_CONTROL_LINE_STATE = const(0x22)
_SEND_BREAK_REQ = const(0x23)

_LINE_CODING_STOP_BIT_1 = const(0)
_LINE_CODING_STOP_BIT_1_5 = const(1)
_LINE_CODING_STOP_BIT_2 = const(2)

_LINE_CODING_PARITY_NONE = const(0)
_LINE_CODING_PARITY_ODD = const(1)
_LINE_CODING_PARITY_EVEN = const(2)
_LINE_CODING_PARITY_MARK = const(3)
_LINE_CODING_PARITY_SPACE = const(4)

_LINE_STATE_DTR = const(1)
_LINE_STATE_RTS = const(2)

_PARITY_BITS_REPR = "NOEMS"
_STOP_BITS_REPR = ("1", "1.5", "2")

# Other definitions
_CDC_VERSION = const(0x0120)  # release number in binary-coded decimal


# CDC data interface definitions
_CDC_ITF_DATA_CLASS = const(0xA)
_CDC_ITF_DATA_SUBCLASS = const(0)
_CDC_ITF_DATA_PROT = const(0)  # no protocol

# Length of the bulk transfer endpoints. Maybe should be configurable?
_BULK_EP_LEN = const(64)

# MicroPython error constants (negated as IOBase.ioctl uses negative return values for error codes)
# these must match values in py/mperrno.h
_MP_EINVAL = const(-22)
_MP_ETIMEDOUT = const(-110)

# MicroPython stream ioctl requests, same as py/stream.h
_MP_STREAM_FLUSH = const(1)
_MP_STREAM_POLL = const(3)

# MicroPython ioctl poll values, same as py/stream.h
_MP_STREAM_POLL_WR = const(0x04)
_MP_STREAM_POLL_RD = const(0x01)
_MP_STREAM_POLL_HUP = const(0x10)


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
        usb_device.device_protocol = 1  # Itf association descriptor

        self._ctrl = CDCControlInterface()
        self._data = CDCDataInterface()
        # The data interface *must* be added immediately after the control interface
        usb_device.add_interface(self._ctrl)
        usb_device.add_interface(self._data)

        # TODO: Add kwargs and call init() with kwargs

    def init(
        self, baudrate=9600, bits=8, parity='N', stop=1, timeout=None, txbuf=64, rxbuf=64, flow=0
    ):
        # Configure the CDC serial port. Note that many of these settings like
        # baudrate, bits, parity, stop don't change the USB-CDC device behavior
        # at all, only the "line coding" as communicated from/to the USB host.

        # Store initial line coding parameters in the USB CDC binary format
        # (there is nothing implemented to further change these from Python
        # code, the USB host sets them.)
        struct.pack_into("<LBBB", self._ctrl._line_coding, 0,
                         baudrate,
                         _STOP_BITS_REPR.index(str(stop)),
                         _PARITY_BITS_REPR.index(parity),
                         bits)

        if flow != 0:
            raise NotImplementedError  # UART flow control currently not supported

        if not (txbuf and rxbuf):
            raise ValueError()  # Buffer sizes are required

        self._data._timeout = timeout
        self._data._wb = Buffer(txbuf)
        self._data._rb = Buffer(rxbuf)

    def is_open(self):
        return self._ctrl.is_open()

    ###
    ### Line State & Line Coding State property getters
    ###

    @property
    def rts(self):
        return bool(self._ctrl._line_state & _LINE_STATE_RTS)

    @property
    def dtr(self):
        return bool(self._ctrl._line_state & _LINE_STATE_DTR)

    # Line Coding Representation
    # Byte 0-3   Byte 4      Byte 5       Byte 6
    # dwDTERate  bCharFormat bParityType  bDataBits

    @property
    def baudrate(self):
        return struct.unpack("<LBBB", self._ctrl._line_coding)[0]

    @property
    def stop_bits(self):
        return _STOP_BITS_REPR[self._ctrl._line_coding[4]]

    @property
    def parity(self):
        return _PARITY_BITS_REPR[self._ctrl._line_coding[5]]

    @property
    def data_bits(self):
        return self._ctrl._line_coding[6]

    def __repr__(self):
        return f"{self.baudrate}/{self.data_bits}{self.parity}{self.stop_bits} rts={self.rts} dtr={self.dtr}"

    ###
    ### Set callbacks for operations initiated by the host
    ###

    def set_break_cb(self, cb):
        self._ctrl.break_cb = cb

    def set_line_state_cb(self, cb):
        self._ctrl.line_state_cb = cb

    def set_line_coding_cb(self, cb):
        self._ctrl.line_coding_cb = cb

    ###
    ### io.IOBase stream implementation
    ###

    def read(self, size=-1):
        return self._data.read(size)

    def readinto(self, b):
        return self._data.readinto(b)

    def write(self, buf):
        return self._data.write(buf)

    def ioctl(self, req, arg):
        return self._data.ioctl(req, arg)


class CDCControlInterface(USBInterface):
    # Implements the CDC Control Interface

    def __init__(self):
        super().__init__(_INTERFACE_CLASS_CDC, _INTERFACE_SUBCLASS_CDC, _PROTOCOL_NONE)

        # Callbacks for particular changes initiated by the host
        self.break_cb = None  # Host sent a "break" condition
        self.line_state_cb = None
        self.line_coding_cb = None

        self._line_state = 0  # DTR & RTS
        # Set a default line coding of 115200/8N1
        self._line_coding = bytearray(b"\x00\xc2\x01\x00\x00\x00\x08")

        self.ep_in = None  # Set when enumeration happens

    def get_itf_descriptor(self, num_eps, itf_idx, str_idx):
        # CDC needs a Interface Association Descriptor (IAD)
        # two interfaces in total
        desc = ustruct.pack(
            "<BBBBBBBB",
            8,
            _ITF_ASSOCIATION_DESC_TYPE,
            itf_idx,
            2,
            _INTERFACE_CLASS_CDC,
            _INTERFACE_SUBCLASS_CDC,
            _PROTOCOL_NONE,
            0,
        )

        itf, strs = super().get_itf_descriptor(num_eps, itf_idx, str_idx)
        desc += itf
        # Append the CDC class-specific interface descriptor
        # see CDC120-track, p20
        desc += ustruct.pack(
            "<BBBH",
            5,  # bFunctionLength
            _CS_DESC_TYPE,  # bDescriptorType
            _CDC_FUNC_DESC_HEADER,  # bDescriptorSubtype
            _CDC_VERSION,
        )  # cdc version

        # CDC-PSTN table3 "Call Management"
        # set to No
        desc += ustruct.pack(
            "<BBBBB",
            5,  # bFunctionLength
            _CS_DESC_TYPE,  # bDescriptorType
            _CDC_FUNC_DESC_CALL_MANAGEMENT,  # bDescriptorSubtype
            0,  # bmCapabilities - XXX no call managment so far
            1,
        )  # bDataInterface - interface 1

        # CDC-PSTN table4 "Abstract Control"
        # set to support line_coding and send_break
        desc += ustruct.pack(
            "<BBBB",
            4,  # bFunctionLength
            _CS_DESC_TYPE,  # bDescriptorType
            _CDC_FUNC_DESC_ABSTRACT_CONTROL,  # bDescriptorSubtype
            0x6,
        )  # bmCapabilities D1, D2
        # CDC-PSTN "Union"
        # set control interface / data interface number
        desc += ustruct.pack(
            "<BBBBB",
            5,  # bFunctionLength
            _CS_DESC_TYPE,  # bDescriptorType
            _CDC_FUNC_DESC_UNION,  # bDescriptorSubtype
            itf_idx,  # bControlInterface
            itf_idx + 1,
        )  # bSubordinateInterface0 (data class itf number)
        return desc, strs

    def get_endpoint_descriptors(self, ep_addr, str_idx):
        self.ep_in = ep_addr | EP_IN_FLAG
        desc = endpoint_descriptor(self.ep_in, "interrupt", 8, 16)
        return (desc, [], (self.ep_in,))

    def handle_interface_control_xfer(self, stage, request):
        # Handle class-specific interface control transfers
        bmRequestType, bRequest, wValue, _, wLength = request
        recipient, req_type, req_dir = split_bmRequestType(bmRequestType)
        if stage == STAGE_SETUP:
            if req_type == REQ_TYPE_CLASS:
                if bRequest == _SET_LINE_CODING_REQ:
                    if wLength == len(self._line_coding):
                        return self._line_coding
                    return False  # wrong length
                elif bRequest == _GET_LINE_CODING_REQ:
                    return self._line_coding
                elif bRequest == _SET_CONTROL_LINE_STATE:
                    if wLength == 0:
                        self._line_state = wValue
                        if self.line_state_cb:
                            self.line_state_cb(wValue)
                        return b""
                    else:
                        return False  # wrong length
                elif bRequest == _SEND_BREAK_REQ:
                    if self.break_cb:
                        self.break_cb(wValue)
                    return b""

        if stage == STAGE_DATA:
            if req_type == REQ_TYPE_CLASS:
                if bRequest == _SET_LINE_CODING_REQ:
                    if self.line_coding_cb:
                        self.line_coding_cb(self._line_coding)

        return True


class CDCDataInterface(USBInterface):
    # Implements the CDC Data Interface

    def __init__(self):
        super().__init__(_CDC_ITF_DATA_CLASS, _CDC_ITF_DATA_SUBCLASS, _CDC_ITF_DATA_PROT)
        self._wb = ()  # Optional write Buffer (IN endpoint), set by CDC.init()
        self._rb = ()  # Optional read Buffer (OUT endpoint), set by CDC.init()
        self._timeout = 1000  # set from CDC.init() as well

        self.ep_in = self.ep_out = None  # Set when enumeration happens

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
        start = time.ticks_ms()
        mv = memoryview(buf)
        while True:
            # TODO: check for failed USB transfers

            # Keep pushing buf into _wb into it's all gone
            nbytes = self._wb.write(mv)
            self._wr_xfer()  # make sure a transfer is running from _wb

            if nbytes == len(mv):
                return len(buf)  # Success

            mv = mv[nbytes:]

            # check for timeout
            if time.ticks_diff(time.ticks_ms(), start) > self._timeout:
                return len(buf) - len(mv)

    def _wr_xfer(self):
        # Submit a new IN transfer from the _wb buffer
        if self.is_open() and self._wb.readable() and not self.xfer_pending(self.ep_in):
            self.submit_xfer(self.ep_in, self._wb.pend_read(), self._wr_cb)

    def _wr_cb(self, ep, res, num_bytes):
        # Whenever an IN transfer ends
        if res == 0:
            self._wb.finish_read(num_bytes)
        self._wr_xfer()

    def _rd_xfer(self):
        # Keep an active OUT transfer to read data from the host,
        # whenever the receive buffer has room for new data
        if self.is_open() and self._rb.writable() and not self.xfer_pending(self.ep_out):
            self.submit_xfer(self.ep_out, self._rb.pend_write(), self._rd_cb)

    def _rd_cb(self, ep, res, num_bytes):
        if res == 0:
            self._rb.finish_write(num_bytes)
        self._rd_xfer()

    def handle_open(self):
        super().handle_open()
        # kick off any transfers that may have queued while the device was not open
        self._rd_xfer()
        self._wr_xfer()

    def read(self, size):
        start = time.ticks_ms()

        # Allocate a suitable buffer to read into
        if size >= 0:
            b = bytearray(size)
        else:
            # for size == -1, return however many bytes are ready
            b = bytearray(self._rb.readable())

        n = self._readinto(b, start)
        if not b:
            return None
        if n < len(b):
            return b[:n]
        return b

    def readinto(self, b):
        return self._readinto(b, time.ticks_ms())

    def _readinto(self, b, start):
        if len(b) == 0:
            return 0

        n = 0
        m = memoryview(b)
        while n < len(b):
            # copy out of the read buffer if there is anything available
            if self._rb.readable():
                n += self._rb.readinto(m if n == 0 else m[n:])
                if n == len(b):
                    break  # Done, exit before we reach the sleep

            if time.ticks_diff(time.ticks_ms(), start) > self._timeout:
                break  # Timed out

            machine.idle()

        return n or None

    def ioctl(self, req, arg):
        if req == _MP_STREAM_POLL:
            return (
                (_MP_STREAM_POLL_WR if (arg & _MP_STREAM_POLL_WR) and self._wb.writable() else 0) |
                (_MP_STREAM_POLL_RD if (arg & _MP_STREAM_POLL_RD) and self._rd.readable() else 0) |
                # using the USB level "open" (i.e. connected to host) for !HUP, not !DTR (port is open)
                (_MP_STREAM_POLL_HUP if (arg & _MP_STREAM_POLL_HUP) and not self.is_open() else 0)
                )
        elif req == _MP_STREAM_FLUSH:
            start = time.ticks_ms()
            # Wait until write buffer contains no bytes for the lower TinyUSB layer to "read"
            while self._wb.readable():
                if not self.is_open():
                    return _MP_EINVAL
                if time.ticks_diff(time.ticks_ms(), start) > self._timeout:
                    return _MP_ETIMEDOUT
                machine.idle()
            return 0

        return _MP_EINVAL
