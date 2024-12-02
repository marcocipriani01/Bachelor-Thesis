import numpy as np

# Sampling rate in Hz
fs = 1024

# Signal frequency in Hz
f = 10

# Signal duration in seconds
T = 1

# Wave type
wave_type = 'sine'

# Offset
offset = 0

# Noise
noise = False

# Create array of samples
t = np.linspace(0, T, T * fs, endpoint=False)

if wave_type == 'sine':
    x = np.sin(2 * np.pi * f * t)
elif wave_type == 'square':
    x = np.sign(np.sin(2 * np.pi * f * t))
elif wave_type == 'triangle':
    x = 2 * np.arcsin(np.sin(2 * np.pi * f * t)) / np.pi
else:
    raise ValueError('Invalid wave type')

# Add noise
if noise:
    x += np.random.normal(0, 0.1, len(x))

# Add offset
x += offset

# print as a C array in a .c file
with open('./Core/Src/dsp_testdata.c', 'w') as f:
    f.write('#include "dsp_testdata.h"\n\n')
    f.write('const int DSP_TEST_DATA_SIZE = ' + str(T * fs) + ';\n\n')
    f.write('const float DSP_TEST_DATA[] = {')
    for i in range(len(x)):
        f.write(str(x[i]) + 'f')
        if i != len(x) - 1:
            f.write(', ')
    f.write('};')
