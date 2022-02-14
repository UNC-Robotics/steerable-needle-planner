# BSD 3-Clause License

# Copyright (c) 2021, The University of North Carolina at Chapel Hill
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#! @author Mengyu Fu

import sys
import open3d as o3d
import copy

colorBank = {
    "0": [1, 0.706, 0],
    "1": [0.3010, 0.7450, 0.9330],
    "2": [0.8500, 0.3250, 0.0980],
    "3": [0.4940, 0.1840, 0.5560],
    "4": [0.4660, 0.6740, 0.1880],
    "5": [0, 0.4470, 0.7410],
    "6": [0.6350, 0.0780, 0.1840],
    "7": [0.75, 0.75, 0],
    "8": [0.5, 0.5, 0.5],
    "size": 9
}

def draw_ptc(ptc):
    ptcs = []

    for i in range(len(ptc)):
        ptcTemp = copy.deepcopy(ptc[i])
        colorIdx = i % colorBank["size"]
        ptcTemp.paint_uniform_color(colorBank[str(colorIdx)])
        ptcs.append(ptcTemp)

    o3d.visualization.draw_geometries(ptcs)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        fileNames = ["../data/output/test_ptcloud.txt", "../data/input/image_points.txt"]
    else:
        fileNames = sys.argv[1:]

    ptcs = []
    for i in range(len(fileNames)):
        ptcFile = fileNames[i]
        ptc = o3d.io.read_point_cloud(ptcFile, format='xyz')
        print("Point cloud {}: ".format(i))
        print(ptc)
        ptcs.append(ptc)

    draw_ptc(ptcs)