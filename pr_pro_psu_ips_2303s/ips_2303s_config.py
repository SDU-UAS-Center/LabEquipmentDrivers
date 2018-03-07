#!/usr/bin/env python
#/****************************************************************************
# PSU driver: RS PRO 2303S driver defines
# Copyright (c) 2018, Martin Skriver <maskr@mmmi.sdu.dk & maskr09@gmail.com
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
Defined values for RS PRO IPS 2303S PSU

2018-03-01 MS Source created
"""

# Debug through terminal
DEBUG = True

# Serial device
# Factory baud is 9600; 57600 or 115200 can be configured to minimize delays
baud_rate_9600 = 9600
baud_rate_115200 = 115200
baud_rate_57600 = 57600

# UART commands to PSU
baud_rate_115200_command = 0
baud_rate_57600_command = 1

# Delays between sending commands and max time for PSU to respond
max_get_data_delay_ms = 80
max_get_param_delay_ms = 500
std_delay_between_msgs_s = 0.020

# Set output
dis_output = 0
en_output = 1

# PSU channel
channel1 = 1
channel2 = 2

# PSU mode. Internal connection for serial and parallel
track_mode_independent = 0
track_mode_series = 1
track_mode_parallel = 2

# PSU memory channelse
mem_channel1 = 1
mem_channel2 = 2
mem_channel3 = 3
mem_channel4 = 4

# Beep sound parameter
sound_off = 0
sound_on = 1
