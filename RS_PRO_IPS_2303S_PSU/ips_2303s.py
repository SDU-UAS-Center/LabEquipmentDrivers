#!/usr/bin/env python
#/****************************************************************************
# PSU driver: RS PRO 2303S driver
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
Source for RS PRO IPS 2303S PSU
Protocol is read from the instruction manual for IPS X303S/D Series DC power supply

2018-02-26 MS Source created
"""
import sys
import serial
from time import sleep
from ips_2303s_config import *

class psuDriver:
    def __init__(self, serial_port, initial_setup_baud = False, initial_baud = baud_rate_9600, target_baud_rate = baud_rate_115200):
        self.serial_port = serial_port
        self.initial_setup_baud = initial_setup_baud
        self.initial_baud = initial_baud 
        self.target_baud_rate = target_baud_rate
        
        self.ser_error = False
        
        # PSU registers
        self.psu_output = dis_output
        self.track_mode = track_mode_independent

        self.init_psu()

    # Setup serial con, disable beep, disable output and set to independent channels
    def init_psu(self):
        if(self.initial_setup_baud):
            self.init_from_factory_settings()
        else:
            self.open_serial_connnection()
            
        self.set_beep(sound_off)
        self.output_status(dis_output)
        self.set_operation_mode(track_mode_independent)

    # Open serial at default baud rate and change to target baud rate
    # This only works if the initial baud rate is selected correctly
    def init_from_factory_settings(self):
        try :
            self.ser = serial.Serial(self.serial_port, self.initial_setup_baud, timeout=0)
        except Exception as e:
            print ("Failed to open serial at: ", self.serial_port)
            self.ser_error = True

        if(self.target_baud_rate == baud_rate_115200):
            self.set_baud_rate(self.baud_rate_115200_command)
            print ("Baud rate have been changed to "+str(baud_rate_115200)+". \nReboot the PSU and use \"open_serial_connnection()\"")            
        elif(self.target_baud_rate == baud_rate_57600):
            self.set_baud_rate(baud_rate_57600_command)
            print ("Baud rate have been changed to "+str(baud_rate_57600)+". \nReboot the PSU and use \"open_serial_connnection()\" ")
        else:
            print ("Baud rate has to be 115200 or 57600")
        sys.exit()

    # Try opening a serial connection to psu        
    def open_serial_connnection(self):
        try :
            self.ser = serial.Serial(self.serial_port, self.target_baud_rate, timeout=0)
        except Exception as e:
            print ("Failed to open serial at: ", self.serial_port)
            self.ser_error = True
    
    # Connect to psu as a direct command prompt 
    def direct_psu_terminal_com(self):
        stopped_by_used = False
        data_out = ''
        sleep(std_delay_between_msgs_s)
        self.show_command_list()
        print('Enter \'exit\' to leave serial mode \nEnter \'HELP?\' to see options again \nExample: \'VSET2:2.987\' to set 2.987 V on output 2 ')

        while not stopped_by_used:
            data_out = raw_input(">> ")
            if data_out == 'exit':
                stopped_by_used = True
            else:
                if (data_out.endswith('?')):
                    self.serial_send_msg(data_out+'\r')
                else:
                    self.serial_send_msg(data_out+'\n')
                sleep(0.5)

                while self.ser.inWaiting() > 0:
                    print (self.get_serial_data(max_get_param_delay_ms))
    
    # Set current limit
    def set_current_output(self, channel, current, check_setvalue = False):
        return_value = False
        self.empty_serial_buffer()
        sleep(std_delay_between_msgs_s)
        
        if(not(channel == channel2 and self.track_mode != track_mode_independent)):
            self.serial_send_msg('ISET'+str(channel)+':'+str("%.3f"%current)+'\n')
    
            if(check_setvalue):
                sleep(std_delay_between_msgs_s)
                var = self.read_set_current_output(channel)
                if(float(var[:len(str("%.3f"%current))]) == current):
                    return_value = True
    
            if DEBUG and check_setvalue and not return_value:
                print('Failed to set current value '+str("%.3f"%current)+ ' to channel '+str(channel))
        else:
            print('Command not allowed: Current can not be set for channel 2 in series or parallel mode! ')
            
        return return_value
        
    # Read current set value
    def read_set_current_output(self, channel):
        sleep(std_delay_between_msgs_s)
        self.empty_serial_buffer()
        self.serial_send_msg('ISET'+str(channel)+'?\r')
        sleep(std_delay_between_msgs_s)
        return self.get_serial_data(max_get_data_delay_ms)

    # Read actual current output
    def read_act_current_output(self, channel):
        sleep(std_delay_between_msgs_s)
        self.empty_serial_buffer()
        self.serial_send_msg('IOUT'+str(channel)+'?\r')
        return (self.get_serial_data(max_get_data_delay_ms))

    # Set voltage for channels with option of checking value but this takes additional time
    def set_voltage_output(self, channel, voltage, check_setvalue = False):
        return_value = False
        self.empty_serial_buffer()
        sleep(std_delay_between_msgs_s)

        if(not(channel == channel2 and self.track_mode == track_mode_parallel)):
            self.serial_send_msg('VSET'+str(channel)+':'+str("%.3f"%voltage)+'\n')
    
            if(check_setvalue):
                sleep(std_delay_between_msgs_s)
                var = self.read_set_voltage_output(channel)
                if(float(var[:len(str("%.3f"%voltage))]) == voltage):
                    return_value = True
    
            if DEBUG and check_setvalue and not return_value:
                print('Failed to set voltage value '+str("%.3f"%voltage)+ ' to channel '+str(channel))
        else:
            print('Command not allowed: Voltage can not be set for channel 2 in parallel mode! ')
        return return_value

    # Read set voltage on psu
    def read_set_voltage_output(self, channel):
        sleep(std_delay_between_msgs_s)
        self.empty_serial_buffer()
        self.serial_send_msg('VSET'+str(channel)+'?\r')
        sleep(std_delay_between_msgs_s)
        return self.get_serial_data(max_get_data_delay_ms)

    # Read measured voltage on output channels on psu
    def read_act_voltage_output(self, channel):
        sleep(std_delay_between_msgs_s)
        self.empty_serial_buffer()
        self.serial_send_msg('VOUT'+str(channel)+'?\r')
        sleep(std_delay_between_msgs_s)
        return self.get_serial_data(max_get_data_delay_ms)

    # Set to parallel, serial or individual cannels in psu
    # 0: Independent; 1:Series; 2:Parallel. Internal connected, read manual for more info
    def set_operation_mode(self, mode):
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('TRACK'+str(mode)+'\n')
        self.track_mode = mode

    # Enable or disable beep function in psu
    def set_beep(self, sound):
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('BEEP'+str(sound)+'\n')

    # Set enable or disable output on psu
    def output_status(self, output):
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('OUT'+str(output)+'\n')

    # Bit(0)->Ch1 0:CC,1CV; Bit(1)->Ch2 0:CC,1CV; Bit(2-3)->01:Independent,11:Series,10:ParallelV; 
    # Bit(4)->0:Beep off,1:Beep on; Bit(5)->NA; Bit(6)->0:Output off,1:Output on; Bit(7)->NA
    def read_status(self):
        self.empty_serial_buffer()
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('STATUS?\r')
        sleep(std_delay_between_msgs_s)
        return self.get_serial_data(max_get_param_delay_ms)

    # Ex: RS PRO  ,IPS-2303S,SN:725B010G2,V2.10
    def read_identification(self):
        sleep(std_delay_between_msgs_s)
        self.empty_serial_buffer()
        self.serial_send_msg('*IDN?\r')
        sleep(std_delay_between_msgs_s)
        return self.get_serial_data(max_get_param_delay_ms)

    # Recall save settings from memory
    def recall_panel_settings(self, memory_no):
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('RCL'+str(memory_no)+'\n')

    # Save settings to memory recalls
    def save_panel_settings(self, memory_no):
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('SAV'+str(memory_no)+'\n')

    # Show commands to send to psu
    def show_command_list(self):
        data_ready = True
        sleep(std_delay_between_msgs_s)
        self.empty_serial_buffer()
        self.serial_send_msg('HELP?\r')
        sleep(std_delay_between_msgs_s)
        while data_ready:
            line_buffer = self.get_serial_data(max_get_param_delay_ms)
            data_ready = False
            if (line_buffer):
                data_ready = True
                print (line_buffer)

    # Print the error msgs from the psu
    def show_error_msgs_list(self):
        data_ready = True
        self.empty_serial_buffer()
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('ERR?\r')
        sleep(std_delay_between_msgs_s)
        while data_ready:
            line_buffer = self.get_serial_data(max_get_data_delay_ms)
            data_ready = False
            if (line_buffer):
                data_ready = True
                print(line_buffer)

    # Set baud rate for psu
    def set_baud_rate(self, baud):
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('BAUD'+str(baud)+'\n')

    # Set to local mode instead of serial. Can not be undone without reboot of psu
    def set_instrument_to_local_mode(self):
        sleep(std_delay_between_msgs_s)
        self.serial_send_msg('LOCAL\n')

    # Return a serial line or the data available in buffer
    def get_serial_data(self, time_out_ms):
        buffer = ''
        message = ''
        timer = 0
        received_line = False
        while time_out_ms > timer and received_line == False:
            while self.ser.inWaiting() and time_out_ms > timer and not received_line:
                buffer = self.ser.read()
                if buffer != '\n' : 
                    message = message + buffer
                    if (buffer == '\r' or buffer == '@'): 
                        received_line = True
                    
            if len(message) < 2 and buffer != '@':
                message = ''
                received_line = False
            timer += 10
            sleep(0.01)
        
        if message != '':
            if DEBUG:
                print (message)
        return message
    
    # Send serial string and debug if DEBUG flag is True
    def serial_send_msg(self, msg):
        self.ser.write(msg)
        if DEBUG:
            print ('Send msg: '+msg)

    # Remove old data from serial buffer
    def empty_serial_buffer(self):
        self.ser.flushInput()