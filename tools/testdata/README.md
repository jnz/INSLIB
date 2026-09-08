# tools/testdata - reference data for the host tools

## imu_tk_xsens_{acc,gyro}.mat.gz

The IMU recording distributed with **IMU-TK** (as `bin/test_data/xsens_*.mat`),
kept here as the reference dataset for `tools/inslib_imu_tk.py`. Gzipped
because it is plain ASCII: 6.3 MB becomes 0.77 MB, and `numpy.loadtxt`
reads `.gz` transparently, so nothing has to unpack it first. The content
is byte-for-byte the original.

Format: one row per sample, `timestamp[s] x y z`, 51175 rows at 100 Hz
(511.7 s). The values are **raw ADC counts**, not SI — roughly 32768 at
rest, which is why the calibration has to be seeded with a bias of 32768
and why the numbers in the test look nothing like the m/s² and rad/s the
INSLIB stream delivers.

### Why it is worth keeping

It is the only check `inslib_imu_tk.py` has against *real* hardware, from
the authors of the method. Synthetic data cannot catch a shared
misconception: if the model were misread, the generator and the solver
would agree with each other and both be wrong. This recording is what
raised the gyro-scale question during the port — imu_tk's own example
application seeds `1/6258` counts per rad/s, and the data says ~1/4770,
which a model-free argument confirms (a gyro cannot integrate to *less*
rotation than gravity demonstrably turned through).

`python/tests/test_imu_tk.py` runs the port against it and gates the
result.

### Provenance and licence

IMU-TK, <https://bitbucket.org/alberto_pretto/imu_tk>, BSD licence:

    Copyright (c) 2014, Alberto Pretto <pretto@diag.uniroma1.it>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in
       the documentation and/or other materials provided with the
       distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
    WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

The method itself:

    D. Tedaldi, A. Pretto, E. Menegatti, "A Robust and Easy to Implement
    Method for IMU Calibration without External Equipments", Proc. IEEE
    International Conference on Robotics and Automation (ICRA), 2014,
    pp. 3042-3049.
