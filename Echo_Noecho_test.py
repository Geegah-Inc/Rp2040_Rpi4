#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 27 18:26:20 2023

@author: geegah
"""
#import modules

import serial

import picoDAQ_Lib   # custom functions for DataAcquisition
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import os
import numpy as np
import time

#%%
savedirname = "/home/geegah/Documents/RaspberryPi_Anuj/DATA/ECHONOECHO_test1/" #change this to whatever you want your directory name to be

if not os.path.exists(savedirname):
    os.makedirs(savedirname)

#folder to dump raw .dat files in 
rawdata_echo_dir = savedirname + "rawdata_echo/"
if not os.path.exists(rawdata_echo_dir):
    os.makedirs(rawdata_echo_dir)

rawdata_noecho_dir = savedirname + "rawdata_noecho/"
if not os.path.exists(rawdata_echo_dir):
    os.makedirs(rawdata_echo_dir)
    
img_save_dir = savedirname + "images/"
if not os.path.exists(img_save_dir):
    os.makedirs(img_save_dir)
    
vid_save_dir = savedirname + "videos/"
if not os.path.exists(vid_save_dir):
    os.makedirs(vid_save_dir)
#%%

ser = serial.Serial('/dev/ttyACM0', 115200)
#%%
mode = 0
#select ADC (1 or 2)
adc = 1
#select timing 25 to 425
timing = 80
packet = str(str(mode)+'.'+str(timing)+'.'+str(adc)+'\n')
print(packet)
    
ser.write(packet.encode())
    #%% Pin assignments AND INITIAL SETUP
GPIO_PINNO_TEST = 27
# VCO, ad4351 PINS, SPI0, DEV0
GPIO_NUM_VCO_LE = 7  #SPI0_CE1 from Rpi

# SPI DAC new device, Configured on SPI0 with dedicated GPIO for C#
GPIO_NUM_DAC_MOSI = 10
GPIO_NUM_DAC_MISO = 9
GPIO_NUM_DAC_CLK = 11
GPIO_NUM_DAC_CE = 8 # STANDARD SPI0, dev0 CE, not used for DAC
GPIO_NUM_DAC_CE0B =22 # Dedicated DAC for 

pinDict_Main = dict(gpio_num_PINNO_TEST = GPIO_PINNO_TEST, 
                    gpio_num_DAC_CE0B = GPIO_NUM_DAC_CE0B,
                    gpio_num_VCO_LE = GPIO_NUM_VCO_LE);
                    
# CLKn frequency for SPI in Hz
CLK_FREQ_SPI_PICO =int(8e6)

# SEtup the GPIO
picoDAQ_Lib.setup_GPIO(pinDict_arg = pinDict_Main)

#Create the sPI object for VCO
# SPI-0, DEV-1, with 
# Get VCO SPi object
spi_obj_vco = picoDAQ_Lib.get_spi_vco()

# VCO settings and registers
freq_target_MHz = 1853
OUTEN_user = 1
PSET_user = 3
regs=picoDAQ_Lib.calc_vco_reg_values(freq_target_MHz,OUTEN_user ,PSET_user)

regshx=[]
for i in range(len(regs)):
    regshx.append("{:#010x}".format(regs[i]))
print("The registers are:", regshx)


# Create list from regs
 # This is basically same as regbytesbyte
regs_list_of_ints  = list(map(picoDAQ_Lib.int2bytes, regs))

#Start writing from the re5 down to reg0
for i in range(len(regs)-1,-1,-1):
    regToWrite_list =regs_list_of_ints[i]
    
    print("writing register",i,regToWrite_list) 
    # Because we connected to LE to CE pin of the pi, and keep CE of the VCO on the board floating
    # LE pin needs to be lowered and made high to load the registers
    #GPIO.output(GPIO_NUM_VCO_LE, 0)
    spi_obj_vco.writebytes(regToWrite_list)
    #GPIO.output(GPIO_NUM_VCO_LE, 1)

# Setup SPI for DAC MAXIM 5252, Bus0, DEV 0 but with dedicated CS GPIO pin at GPIO_NUM_DAC_CE0B

DAC_val = 3815
spi_obj_dac = picoDAQ_Lib.get_spi_dac_MAXIM5123()
picoDAQ_Lib.writeDAC_MAXIM5123(spi_obj_dac, DAC_val, GPIO_NUM_DAC_CE0B)

# GET PICO SPI object
spi_obj_pico = picoDAQ_Lib.get_spi_pico(CLK_FREQ_SPI_PICO)

# Block read Size
n_bytes_block_arg=1<<12

#%%
# number of dummy frames
N_dummy_frames=2

# Flag to SAve figures
flagSaveFig = True
# flag to Save Data
flagSaveData = True
# Create the folder if necessary
#%% Do A Dummy Read

# Number of Frames

N_baseline_frames = 2

# Add 2 dummy frames at the beginning
frames_I_baseline, frames_Q_baseline , bufferSignature_1D_baseline, missedBlobCount_1D_baseline = picoDAQ_Lib.getFrames_I_and_Q(spi_ob_arg=spi_obj_pico, nFrames = N_baseline_frames+N_dummy_frames)
#%%sweeping through frames, mode = 2

time_i = time.time()
I = []
Q = []
Mag = []
Phase = []

frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
    
frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
for jj in range(100):
    
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    
    fname = rawdata_echo_dir + "frame"+str(jj)+".dat"
    picoDAQ_Lib.writeFile(fname, frames_data_nD_baseline)
    F_IMG_I, F_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
    I_AE_VOLTS, Q_AE_VOLTS = picoDAQ_Lib.convertADCToVolts(F_IMG_I, F_IMG_Q)
    
    I.append(I_AE_VOLTS)
    Q.append(Q_AE_VOLTS)
    
    #I.append(F_IMG_I)
    #Q.append(F_IMG_Q)
    
    MAG_AE_i  = np.sqrt(np.square(I_AE_VOLTS)+np.square(Q_AE_VOLTS))
    PHASE_AE_i = np.arctan2(I_AE_VOLTS, Q_AE_VOLTS)
    
    
    Phase.append(PHASE_AE_i)
    time.sleep(0.001)
    print("DONE step = "+str(jj))

time_f = time.time()
del_time = time_f - time_i
fps = 100/del_time

print("average fps = " + str(fps))
print("DONE ACQUIRING BASELINE I AND Q ECHO")


  #%%
plt.close("all")
spi_obj_vco.close()  #Close the SPI object
spi_obj_pico.close() # Close the SPI object
spi_obj_dac.close()
GPIO.setwarnings(True)
GPIO.cleanup()
                                          
print("Done")


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    