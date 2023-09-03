from utils import Buffer

# TODO: Makes this a test which can be integrated somewhere

b = Buffer(16)

# Check buffer is empty
assert b.readable() == 0
assert b.writable() == 16

# Single write then read
w = b.pend_write()
assert len(w) == 16
w[:8] = b'12345678'
b.finish_write(8)

# Empty again
assert b.readable() == 8
assert b.writable() == 8

r = b.pend_read()
assert len(r) == 8
assert r == b'12345678'
b.finish_read(8)

# Empty buffer again
assert b.readable() == 0
assert b.writable() == 16

# Single write then split reads
b.write(b'abcdefghijklmnop')
assert b.writable() == 0  # full buffer

r = b.pend_read()
assert r == b'abcdefghijklmnop'
b.finish_read(2)

r = b.pend_read()
assert r == b'cdefghijklmnop'
b.finish_read(3)

# write to end of buffer
b.write(b'AB')

r = b.pend_read()
assert r == b'fghijklmnopAB'

# write while a read is pending
b.write(b'XY')

# previous pend_read() memoryview should be the same
assert r == b'fghijklmnopAB'

b.finish_read(4)
r = b.pend_read()
assert r == b'jklmnopABXY'  # four bytes consumed from head, one new byte at tail

# read while a write is pending
w = b.pend_write()
assert len(w) == 5
r = b.pend_read()
assert len(r) == 11
b.finish_read(3)
w[:2] = b'12'
b.finish_write(2)

# Expected final state of buffer
assert b.read() == b'mnopABXY12'

# Now buffer is empty again
assert b.read() == b''
assert b.readable() == 0
assert b.writable() == 16

print('Done!')