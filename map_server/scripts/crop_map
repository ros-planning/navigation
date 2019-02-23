#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

from __future__ import print_function

import sys
import yaml
from PIL import Image
import math

def find_bounds(map_image):
    x_min = map_image.size[0]
    x_end = 0
    y_min = map_image.size[1]
    y_end = 0
    pix = map_image.load()
    for x in range(map_image.size[0]):
        for y in range(map_image.size[1]):
            val = pix[x, y]
            if val != 205:  # not unknown
                x_min = min(x, x_min)
                x_end = max(x, x_end)
                y_min = min(y, y_min)
                y_end = max(y, y_end)
    return x_min, x_end, y_min, y_end

def computed_cropped_origin(map_image, bounds, resolution, origin):
    """ Compute the image for the cropped map when map_image is cropped by bounds and had origin before. """
    ox = origin[0]
    oy = origin[1]
    oth = origin[2]

    # First figure out the delta we have to translate from the lower left corner (which is the origin)
    # in the image system
    dx = bounds[0] * resolution
    dy = (map_image.size[1] - bounds[3]) * resolution

    # Next rotate this by the theta and add to the old origin

    new_ox = ox + dx * math.cos(oth) - dy * math.sin(oth)
    new_oy = oy + dx * math.sin(oth) + dy * math.cos(oth)

    return [new_ox, new_oy, oth]

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: %s map.yaml [cropped.yaml]" % sys.argv[0], file=sys.stderr)
        sys.exit(1)

    with open(sys.argv[1]) as f:
        map_data = yaml.safe_load(f)

    if len(sys.argv) > 2:
        crop_name = sys.argv[2]
        if crop_name.endswith(".yaml"):
            crop_name = crop_name[:-5]
        crop_yaml = crop_name + ".yaml"
        crop_image = crop_name + ".pgm"
    else:
        crop_yaml = "cropped.yaml"
        crop_image = "cropped.pgm"

    map_image_file = map_data["image"]
    resolution = map_data["resolution"]
    origin = map_data["origin"]

    map_image = Image.open(map_image_file)

    bounds = find_bounds(map_image)

    # left, upper, right, lower
    cropped_image = map_image.crop((bounds[0], bounds[2], bounds[1] + 1, bounds[3] + 1))

    cropped_image.save(crop_image)
    map_data["image"] = crop_image
    map_data["origin"] = computed_cropped_origin(map_image, bounds, resolution, origin)
    with open(crop_yaml, "w") as f:
        yaml.dump(map_data, f)

