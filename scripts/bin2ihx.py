import argparse

parser = argparse.ArgumentParser()
parser.add_argument('infile')
parser.add_argument('outfile')
parser.add_argument('-r', '--record_size', type=int, default=32, help='bytes of binary data per line of text')
parser.add_argument('-s', '--start_address', type=int, default=0, help='where the binary data will be loaded into memory')
args = parser.parse_args()

REC_SIZE = args.record_size

with open(args.infile, mode="rb") as f:
  data = f.read()

with open(args.outfile, mode="w") as f:
  index = 0
  while index < len(data):
    rec = data[index:index+REC_SIZE]
    size = len(rec)

    # TODO add support for 24/32 bit addresses using 02/04 record types
    address = index + args.start_address
    f.write(f':{size:02X}{address:04X}00')
    checksum = size + (address >> 8) + (address & 0xFF)
    index += REC_SIZE

    for byte in rec:
      f.write(f'{byte:02X}')
      checksum += byte

    checksum = -checksum & 0xFF
    f.write(f'{checksum:02X}\n')
  f.write(':00000001FF\n')
