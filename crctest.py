

with open("/home/skyler/git/raspberrypi/pico-examples/build/hello_world/serial/hello_serial.bin", "rb") as f:
    data = f.read(256)

expected_crc = data[-4:]
data_to_crc = data[:-4]

print(expected_crc.hex())

def rev(x):
    return int.from_bytes(x.to_bytes(4, "little"), "big")

def dump_regs(r):
    for i in range(0, 12):
        print(f"r{i}: {r[i]:08x}")


r = [0] * 12
sram_base = 0x20041f00
r[0] = sram_base # index into data_to_crc
r[1] = 252 # length of data_to_crc
r[2] = 0xffff_ffff #seed

r[1] = r[1] + r[0] # adds r1, r1, r0
r[5] = 0x04c11db7  # ldr r5, =0x04c11db7 polynomial

while r[0] < r[1]:                    # cmp r0, r0
                                      # blt byte_loop
                                      # byte_loop:
    print(f"CRC index {r[0]-sram_base:08x}")
    dump_regs(r)

    r[4] = data_to_crc[r[0]-sram_base]          # ldrb r4, [r0, #0]
    r[3] = rev(r[2])                  # rev r3, r2
    r[4] = r[4] ^ r[3]                # eor r4, r4, r3
    r[4] = (r[4] << 24) & 0xffff_ffff # lsl r4, r4, #24
    r[3] = 8                          # mov r3, #8
    while r[3] > 0:
        carry = r[4] & 0x8000_0000
        r[4] = (r[4] << 1) & 0xffff_ffff# lsl r4, r4, #1
        if carry:
            r[4] = r[4] ^ r[5]        # eor r4, r4, r5
        r[3] = r[3] - 1               # sub r3, r3, #1
    r[2] = (r[2] << 8) & 0xffff_ffff  # lsl r2, r2, #8
    r[2] = r[2] ^ r[4]                # eor r2, r2, r4
    r[0] = r[0] + 1                    # add r0, r0, #1

r[0] = r[2] + 0 

dump_regs(r)
print(f"Expected CRC: {expected_crc.hex()}")
print(f"Calculated CRC: {r[0]:08x}")

