#!/usr/bin/env python
#/****************************************************************************
# PSU driver: Example code for RS PRO 2303S driver
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
Example for RS PRO IPS 2303S PSU

2018-03-01 MS Source created
"""

from time import sleep
from ips_2303s import *

# Serial device
device = "/dev/ttyUSB0"

# Set values to the four mamory recalls on psu
def use_psu_memory(psu_instance):
    psu_inst.output_status(dis_output)
    psu_inst.set_operation_mode(track_mode_independent)
    
    # Set current and voltage values to memory 1
    if not (psu_instance.set_current_output(channel1, 1.200, True) and 
            psu_instance.set_voltage_output(channel1, 3.300, True) and 
            psu_instance.set_current_output(channel2, 1.000, True) and 
            psu_instance.set_voltage_output(channel2, 5.000, True)):
        print ("Value have not been saved correctly in psu")
        sys.exit()

    psu_instance.save_panel_settings(mem_channel1)
    sleep(2)

    # Set current and voltage values to memory 2
    psu_instance.set_current_output(channel1, 1.500, True)
    psu_instance.set_voltage_output(channel1, 12.000, True)
    psu_instance.set_current_output(channel2, 0.500, True)
    psu_instance.set_voltage_output(channel2, 1.200, True)
    psu_instance.save_panel_settings(mem_channel2)

    # Enable output and switch between memory values
    psu_inst.recall_panel_settings(mem_channel1)
    psu_inst.output_status(en_output)
    sleep(2)
    psu_inst.recall_panel_settings(mem_channel2)
    psu_inst.output_status(en_output)
    sleep(2)
    psu_inst.recall_panel_settings(mem_channel1)
    psu_inst.output_status(en_output)
    sleep(2)
    psu_inst.output_status(dis_output)

    print ('Error msgs log:' )
    psu_inst.show_error_msgs_list()

# Set current and voltage values and change them while output is enabled
def run_sequence(psu_instance):
    # Disable output and set to independt psu channels
    psu_instance.output_status(dis_output)
    psu_instance.set_operation_mode(track_mode_independent)
    
    # Set current output limit and voltage and check if value is stored in PSU
    if not (psu_instance.set_current_output(channel2, 0.535, True)):
        print ("Current value have not been saved correctly in psu")
        sys.exit()
    if not (psu_instance.set_voltage_output(channel2, 2.234, True)):
        print ("Voltage value have not been saved correctly in psu")
        sys.exit()
    
    # Read current and voltage set values
    print ('Current set value cannel 2: ' + psu_instance.read_set_current_output(channel2))
    print ('Voltage set value cannel 2: ' + psu_instance.read_set_voltage_output(channel2))

    # Enable output
    psu_instance.output_status(en_output)

    # Read actual current and voltage values
    print ('Current actual value cannel 2: ' + psu_instance.read_act_current_output(channel2))
    print ('Voltage actual value cannel 2: ' + psu_instance.read_act_voltage_output(channel2))

    # Wait a few seconds and change the output values without check (Faster than with check)
    sleep(2)
    psu_instance.set_current_output(channel2, 1.200)
    psu_instance.set_voltage_output(channel2, 24.000)
    sleep(2)
    psu_instance.set_current_output(channel2, 1.000)
    psu_instance.set_voltage_output(channel2, 12.400)
    sleep(2)
    psu_instance.set_current_output(channel2, 1.500)
    psu_instance.set_voltage_output(channel2, 5.000)
    psu_instance.output_status(dis_output)
    sleep(2)
    print ('Error msgs log:' )
    psu_inst.show_error_msgs_list()

# Serial connected internal in psu. Lim 60 V and 3 A
# Use ch1(master) + for positive terminal, ch2(slave) - for negative terminal
# For +/- connection, use ch1(master) - for ground potential
def use_serial_con_psu(psu_instance):
    # Disable output and set to seriel connection
    psu_instance.output_status(dis_output)
    psu_instance.set_operation_mode(track_mode_series)
    
    psu_instance.set_current_output(channel1, 1.200)
    psu_instance.set_voltage_output(channel1, 24.000)

    psu_instance.output_status(en_output)
    sleep(2)

    psu_instance.set_current_output(channel2, 1.500) # Not allowed
    psu_instance.set_voltage_output(channel1, 12.000)
    sleep(2)

    psu_instance.output_status(dis_output)
    sleep(2)

    print ('Error msgs log:' )
    psu_inst.show_error_msgs_list()

# Parallel connected internal in psu. Lim 30 V and 6 A
# Use ch1(master) terminals
def use_para_con_psu(psu_instance):
    # Disable output and set to parallel connection
    psu_instance.output_status(dis_output)
    psu_instance.set_operation_mode(track_mode_parallel)
    
    psu_instance.set_current_output(channel1, 2.200)
    psu_instance.set_voltage_output(channel1, 24.000)
    psu_instance.set_current_output(channel2, 1.200)
    sleep(2)

    psu_instance.output_status(en_output)
    sleep(2)
    psu_instance.set_current_output(channel1, 1.200)
    psu_instance.set_voltage_output(channel1, 12.000)
    sleep(2)
    psu_instance.output_status(dis_output)
    sleep(2)

    print ('Error msgs log:' )
    psu_inst.show_error_msgs_list()

# Communicate directly with psu
def direct_connection():
    psu_inst.direct_psu_terminal_com()        

# Show status, identification and error msg list
def show_internal_parameters():
    print (psu_inst.read_status())
    print (psu_inst.read_identification())    
    print (psu_inst.show_error_msgs_list())

# Main function.    
if __name__ == '__main__':

    try:
        psu_inst = psuDriver(device, False)
        # Run this first time psu is in use
        #psu_inst = psuDriver("/dev/ttyUSB0", True, baud_rate_9600, baud_rate_115200) 
    except Exception as e:
        print ("Something went wrong! ")

# Uncomment to execute method
#    show_internal_parameters()
#    direct_connection()
#    run_sequence(psu_inst)
#    use_serial_con_psu(psu_inst)
#    use_para_con_psu(psu_inst)
    use_psu_memory(psu_inst)
#    self.set_instrument_to_local_mode() # Requires reboot of psu to get connection again