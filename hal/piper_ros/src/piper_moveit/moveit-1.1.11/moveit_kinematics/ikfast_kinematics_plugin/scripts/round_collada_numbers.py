#! /usr/bin/env python

"""
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
"""
# Author: Dave Coleman
# Desc:   Rounds all the numbers to <decimal places> places

from __future__ import print_function

from lxml import etree
import shlex
import sys


def doRound(values, decimal_places):
    num_vector = shlex.split(values)
    new_vector = []

    for num in num_vector:
        new_num = round(float(num), decimal_places)
        print("Old:", num, "New:", new_num)
        new_vector.append(str(new_num))

    new = " ".join(new_vector)
    # print('Original:', values, '  Updated: ', new)

    return new


# -----------------------------------------------------------------------------

if __name__ == "__main__":

    # Check input arguments
    try:
        input_file = sys.argv[1]
        output_file = sys.argv[2]
        decimal_places = int(sys.argv[3])
        assert len(sys.argv) < 5  # invalid num-arguments
    except:
        print(
            "\nUsage: round_collada_numbers.py <input_dae> <output_dae> <decimal places>"
        )
        print("Rounds all the numbers to <decimal places> places\n")
        sys.exit(-1)

    print("\nCollada Number Rounder")
    print("Rounding numbers to", decimal_places, "decimal places\n")

    namespace = "http://www.collada.org/2008/03/COLLADASchema"
    dom = etree.parse(input_file)

    # find elements of particular name
    elements = dom.xpath("//ns:translate", namespaces={"ns": namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text, decimal_places)

    # find elements of particular name
    elements = dom.xpath("//ns:rotate", namespaces={"ns": namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text, decimal_places)

    # find elements of particular name
    elements = dom.xpath("//ns:min", namespaces={"ns": namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text, decimal_places)

    # find elements of particular name
    elements = dom.xpath("//ns:max", namespaces={"ns": namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text, decimal_places)

    # find elements of particular name
    elements = dom.xpath("//ns:float", namespaces={"ns": namespace})
    for i in range(len(elements)):
        elements[i].text = doRound(elements[i].text, decimal_places)

    # save changes
    with open(output_file, "wb") as f:
        dom.write(f, encoding="utf-8")
