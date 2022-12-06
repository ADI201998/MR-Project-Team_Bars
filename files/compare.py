import numpy as np

f1 = open("ceres_input_multiViewAdjuster-new.txt", "r")
f2 = open("ceres_input_multiViewAdjuster.txt", "r")

l01 = np.array([float(l) for l in f1.readline().split()])
l02 = np.array([float(l) for l in f2.readline().split()])

print(l01, l02)
print(np.abs(l01-l02))

l11 = np.array([float(l) for l in f1.readline().split()])
l12 = np.array([float(l) for l in f2.readline().split()])

print()
print(np.abs(l11-l12))

l21 = np.array([float(l) for l in f1.readline().split()])
l22 = np.array([float(l) for l in f2.readline().split()])

print()
print(np.abs(l21-l22))

b1 = []
b2 = []
for i in range(3, 244):
    b1.append([float(l) for l in f1.readline().split()])
    b2.append([float(l) for l in f2.readline().split()])
b1 = np.array(b1)
b2 = np.array(b2)
print()
bdiff = np.abs(b1-b2)
print(np.mean(bdiff, axis=0))
print(np.min(bdiff, axis=0))
print(np.max(bdiff, axis=0))

b1 = []
b2 = []
for i in range(244, 8920):
    b1.append([float(l) for l in f1.readline().split()])
    b2.append([float(l) for l in f2.readline().split()])
b1 = np.array(b1)
b2 = np.array(b2)
#print(b2[0], b2[-1])
print()
bdiff = np.abs(b1-b2)
print(np.mean(bdiff, axis=0))
print(np.min(bdiff, axis=0))
print(np.max(bdiff, axis=0))

kl1 = []
kl2 = []
for i in range(8920, 17596):
    kl1.append([float(l) for l in f1.readline().split()])
    kl2.append([float(l) for l in f2.readline().split()])
kl1 = np.array(kl1)
kl2 = np.array(kl2)
#print(kl2[0], kl2[-1])
print()
kldiff = np.abs(kl1-kl2)
print(np.mean(kldiff, axis=0))
print(np.min(kldiff, axis=0))
print(np.max(kldiff, axis=0))

lb1 = []
lb2 = []
for i in range(17596, 26272):
    lb1.append([float(l) for l in f1.readline().split()])
    lb2.append([float(l) for l in f2.readline().split()])
lb1 = np.array(lb1)
lb2 = np.array(lb2)
#print(lb2[0], lb2[-1])
print()
lbdiff = np.abs(lb1-lb2)
print(np.mean(lbdiff, axis=0))
print(np.min(lbdiff, axis=0))
print(np.max(lbdiff, axis=0))

lc1 = []
lc2 = []
for i in range(26272, 36394):
    lc1.append([float(l) for l in f1.readline().split()])
    lc2.append([float(l) for l in f2.readline().split()])
lc1 = np.array(lc1)
lc2 = np.array(lc2)
#print(lc2[0])
#print(lc2[-1])
print()
lcdiff = np.abs(lc1-lc2)
print(np.mean(lcdiff, axis=0))
print(np.min(lcdiff, axis=0))
print(np.max(lcdiff, axis=0))

l21 = np.array([float(l) for l in f1.readline().split()])
l22 = np.array([float(l) for l in f2.readline().split()])

print()
print(np.abs(l21-l22))

ld1 = []
ld2 = []
for i in range(36395, 39287):
    ld1.append([float(l) for l in f1.readline().split()])
    ld2.append([float(l) for l in f2.readline().split()])
ld1 = np.array(ld1)
ld2 = np.array(ld2)
#print(ld2[0])
#print(ld2[-1])
print()
lddiff = np.abs(ld1-ld2)
print(np.mean(lddiff, axis=0))
print(np.min(lddiff, axis=0))
print(np.max(lddiff, axis=0))