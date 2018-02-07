#!/usr/bin/python

# Here is a web site where this class comes from:
# It contains a class for both 2D and 3D points
#http://www1.cmc.edu/pages/faculty/alee/python/points.py

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

class Point(object):
    # Documentation string
    """
       The class Point represents a 3D point
       Class attributes:    points
       Instance attributes: x
                            y
                            z
    """

    # Class attributes:
    #
    # To access a class attribute, use dot notation, e.g., Point.points
    # as is done in __init__ below.
    # Note: there is only one copy of a class attribute
    #       whereas there is a copy of instance attribute in
    #       every Point instance.
    points = []

    # Constructors
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        Point.points.append(self)

    def __init__(self, x=0, y=0 , z = 0):
        self.x = x
        self.y = y
        self.z = z
        Point.points.append(self)

    # toString method in Java for those who are familiar with Java
    # Generating a string representation of a Point object
    def __str__(self):
       return '(%g, %g, %g)' % (self.x, self.y, self.z)
       # return '(' + str(self.x) + ', ' + str(self.y) + ')'

    # Special names methods. . .
    # With this method defined, we can use + to add two point objects
    # as in p1 + p2 which is equivalent to p1.__add__(p2)
    # See http://docs.python.org/ref/specialnames.html for others
    # Also see http://docs.python.org/reference/ for general language
    # reference
    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    # With this method defined, two point objects can be compared with
    # >, <, and ==.
    def __cmp__(self, other):
        # compare them using the x values first
        if self.x > other.x: return 1
        if self.x < other.x: return -1

        # x values are the same... check y values
        if self.y > other.y: return 1
        if self.y < other.y: return -1

        # y values are the same... check z values
        if self.z > other.z: return 1
        if self.z < other.z: return -1

        # y values are the same too. . . it's a tie
        return 0


    # Other general methods
    def distance_from_origin(elf):
        return math.sqrt(elf.x * elf.x + elf.y * elf.y + elf.z * elf.z)

    def distance(self, other):
        dx = math.fabs(self.x - other.x)
        dy = math.fabs(self.y - other.y)
        dz = math.fabs(self.z - other.z)
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def isIn1stQuad(self):
        return (self.x > 0) and (self.y > 0)
        
    # class Point ends here
    #*********************************************************
