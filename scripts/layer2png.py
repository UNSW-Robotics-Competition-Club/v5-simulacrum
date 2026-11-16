import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ppm2png import write_png
d=open(sys.argv[1],"rb").read(); base=int(sys.argv[3],0); w=512
rgb=bytearray()
for y in range(272):
    for x in range(480):
        o=base+(y*w+x)*4; rgb+=bytes([d[o+2],d[o+1],d[o]])
write_png(sys.argv[2],480,272,bytes(rgb),1)
