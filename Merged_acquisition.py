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


#%%
savedirname = "/home/geegah/Documents/RaspberryPi_Anuj/DATA/MODE2/" #change this to whatever you want your directory name to be

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

def acq_MODE(mode = 0, time = 125):
    # open microcontroller serial port
    # change the port name accordingly
    #use "python -m serial.tools.list_ports" in terminal to view connected ports
    
    #ser = serial.Serial('/dev/tty.usbmodem14401', 115200)
    ser = serial.Serial('/dev/ttyACM0', 115200)
    
    #select mode for acquisition:
        #Mode 0 = echo acquisition
        #Mode 1 = no-echo acquisition
        #Mode 2 = Sweep through time settings 
    mode = mode
    #select ADC (1 or 2)
    adc = 1
    #select timing 25 to 425
    timing = time
    
    packet = str(str(mode)+'.'+str(timing)+'.'+str(adc)+'\n')
    print(packet)
    
    ser.write(packet.encode())
    ser.close()

#%%
ser = serial.Serial('/dev/ttyACM0', 115200)
#%%
mode = 2
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

#%% Manually reset
# Save the Baseline Frames  
import time
frames = 2
time_i = time.time()
for jj in range(frames):
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    
        
    F_IMG_I, F_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
    I_AE_VOLTS, Q_AE_VOLTS = picoDAQ_Lib.convertADCToVolts(F_IMG_I, F_IMG_Q)
    MAG_AE  = np.sqrt(np.square(I_AE_VOLTS)+np.square(Q_AE_VOLTS))
    PHASE_AE = np.arctan2(I_AE_VOLTS, Q_AE_VOLTS)
    time.sleep(0.001)
    print("DONE frame = "+str(jj))


baseline_echo_fname = savedirname + "baseline_echo.dat"
picoDAQ_Lib.writeFile(baseline_echo_fname, frames_data_nD_baseline)

time_f = time.time()
del_time = time_f - time_i
fps = frames/del_time
print("average fps = " + str(fps))
print("DONE ACQUIRING BASELINE I AND Q ECHO")

#%%REAL TIME ACQUISITION

numFrames = 100
numDigits = len(str(numFrames)) #this is for file save names
#128x128 pixels displays very small with OpenCV
#want to rescale up to a larger image size of 512 x 512 pixels 


#initialize image window
fig2,ax2 = plt.subplots(1)
fig2.set_size_inches(2,2)
mytitle = fig2.suptitle('Real-time Ultrasonic Images:  ')
   
im2 = np.flipud(np.rot90(Q_AE_VOLTS,1))
pos201 = ax2.imshow(im2, vmin = -0.1, vmax = 0.1, interpolation = 'bilinear')
fig2.colorbar(pos201)
base_title ='Real-time Ultrasonic image (V):  '
#time how long this takes125
t1 = time.time()
for cf in range(numFrames):
    #read an image
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                           n_bytes_block_arg=n_bytes_block_arg)
    S_IMG_I, S_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)

    #write file
    img_file_name = rawdata_echo_dir+"frame"+str(cf).zfill(numDigits)+".dat"
    picoDAQ_Lib.writeFile(img_file_name, frames_data_nD_baseline) 
    I_SE_VOLTS, Q_SE_VOLTS = picoDAQ_Lib.convertADCToVolts(S_IMG_I, S_IMG_Q)
    MAG_SE  = np.sqrt(np.square(I_SE_VOLTS)+np.square(Q_SE_VOLTS))
    PHASE_SE = np.arctan2(I_SE_VOLTS, Q_SE_VOLTS)
     
    MAG = MAG_SE - MAG_AE
    PHASE = PHASE_SE - PHASE_AE
    RCOEF = MAG_SE/MAG_AE
    #plot only Q
   
    sub_Q = Q_SE_VOLTS - Q_AE_VOLTS
    sub_I = I_SE_VOLTS - I_AE_VOLTS
   
    mytitle.set_text(base_title+str(cf)+' of ' +str(numFrames-1))
    sub_image = np.flipud(np.rot90(MAG,1))
    pos201.set_data(sub_image)
    pos201.set_clim(sub_image.min(), sub_image.max())
    plt.pause(0.001)

    #redraw
    fig2.canvas.draw_idle()
    fig2.savefig(img_save_dir+'plot'+str(cf)+'.png')

#%%ECHO AND NO_ECHO BASELINE

import time

time_i = time.time()

#ECHO
acq_MODE(mode = 1, time = 125)   
time.sleep(0.01)
frames_data_nD_baseline1,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
F_IMG_I, F_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline1)
I_AE_VOLTS, Q_AE_VOLTS = picoDAQ_Lib.convertADCToVolts(F_IMG_I, F_IMG_Q)


baseline_echo_fname = savedirname + "baseline_echo.dat"
picoDAQ_Lib.writeFile(baseline_echo_fname, frames_data_nD_baseline1)
time.sleep(0.1)

acq_MODE(mode = 1, time = 450)   
time.sleep(0.01) 
frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
FN_IMG_I, FN_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
I_ANE_VOLTS, Q_ANE_VOLTS = picoDAQ_Lib.convertADCToVolts(FN_IMG_I, FN_IMG_Q)

frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
FN_IMG_I, FN_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
I_ANE_VOLTS, Q_ANE_VOLTS = picoDAQ_Lib.convertADCToVolts(FN_IMG_I, FN_IMG_Q)

frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
FN_IMG_I, FN_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
I_ANE_VOLTS, Q_ANE_VOLTS = picoDAQ_Lib.convertADCToVolts(FN_IMG_I, FN_IMG_Q)

baseline_noecho_fname = savedirname + "baseline_noecho.dat"
picoDAQ_Lib.writeFile(baseline_noecho_fname, frames_data_nD_baseline)

#%%

hist_plot([I_AE_VOLTS, I_ANE_VOLTS], string = ["AIR I mod0 t125ns", "AIR I mod0 t450ns"])

#%%

m_list = [I_AE_VOLTS, I_ANE_VOLTS, Q_AE_VOLTS, Q_ANE_VOLTS]
str_list = ["I mod0 t125ns", "I mod0 t450ns","Q mod0 t125ns", "Q mod0 t450ns"]
#%%
multi_image(m_list, str_list)


#%%
def multi_image(mat_list, str_list, vmin = 0, vmax  = 0.2):
    num1 = len(mat_list)
    if num1%2 ==0:
        if num1 !=2:
            row = num1//2
            col = num1//2
            
        else:
            row = num1//2
            col = num1//2 +1
    else:
        
        row = (num1//2) + 1
        col = (num1//2) + 1
        
    fig  = plt.figure()
    
    for jj in range(num1):
        fig.add_subplot(row,col,jj+1)
        plt.imshow(mat_list[jj], vmin = vmin , vmax = vmax)
        plt.xlabel("Columns")
        plt.ylabel("Rows")
        plt.title(str_list[jj])
        plt.colorbar()

    plt.tight_layout()
    plt.show()


#%%
MAG_A = np.sqrt(np.square(I_AE_VOLTS - I_ANE_VOLTS)+np.square(Q_AE_VOLTS - Q_ANE_VOLTS))
PHASE_A = np.arctan2(Q_AE_VOLTS - Q_ANE_VOLTS, I_AE_VOLTS - I_ANE_VOLTS)



time_f = time.time()
del_time = time_f - time_i

print("DONE ACQUIRING BASELINE I AND Q ECHO and NOECHO")

#%%
plt.plot(frames_data_nD_baseline1, label = 'MODE 1, TIME = 125ns')
plt.plot(frames_data_nD_baseline, 'r', label = 'MODE 1, TIME = 440ns')
plt.title("BYTES DATA")
plt.legend()
#%%ECHO AND NO_ECHO SAMPLES
#ECHO
acq_MODE(mode = 0)   
time.sleep(0.01)
frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
S_IMG_I, S_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline1)
I_SE_VOLTS, Q_SE_VOLTS = picoDAQ_Lib.convertADCToVolts(S_IMG_I, S_IMG_Q)

sample_echo_fname = savedirname + "sample_echo.dat"
picoDAQ_Lib.writeFile(sample_echo_fname, frames_data_nD_baseline1)


acq_MODE(mode = 0, time = 440)   
time.sleep(0.01)
frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                   n_bytes_block_arg=n_bytes_block_arg)
SN_IMG_I, SN_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
I_SNE_VOLTS, Q_SNE_VOLTS = picoDAQ_Lib.convertADCToVolts(SN_IMG_I, SN_IMG_Q)


sample_noecho_fname = savedirname + "sample_noecho.dat"
picoDAQ_Lib.writeFile(sample_noecho_fname, frames_data_nD_baseline)

MAG_S = np.sqrt(np.square(I_SE_VOLTS - I_SNE_VOLTS)+np.square(Q_SE_VOLTS - Q_SNE_VOLTS))
PHASE_S = np.arctan2(Q_SE_VOLTS - Q_SNE_VOLTS, I_SE_VOLTS - I_SNE_VOLTS)

print("DONE ACQUIRING SAMPLE AND Q ECHO and NOECHO")
#%%
plt.plot(frames_data_nD_baseline1, label = 'MODE 0, TIME = 125ns')
plt.plot(frames_data_nD_baseline, 'r', label = 'MODE 0, TIME = 440ns')
plt.title("BYTES DATA")
plt.legend()
#%%PLOTIING 
RCoef = MAG_S-MAG_A
Phase = PHASE_S - PHASE_A
plt.imshow(RCoef , interpolation = "bilinear")
plt.title("Reflection coefficient")
plt.xlabel("Columns")
plt.ylabel("Rows")
plt.colorbar()


  #%%
plt.close("all")
spi_obj_vco.close()  #Close the SPI object
spi_obj_pico.close() # Close the SPI object
spi_obj_dac.close()
GPIO.setwarnings(True)
GPIO.cleanup()
                                          
print("Done")


#%%

def hist_plot(Mainlist, bins = 90, string = []):
    fig, ax = plt.subplots(1,1)
    ax.set_xlabel("Signal")
    ax.set_ylabel("No. of pixels")
    ax.set_title("Distribution of signal")
    
    flat_list = []
    for jj in Mainlist:
        bb = np.array(jj)
        flat_list.append(bb)
        
    for jj in range(len(flat_list)):
        _ = ax.hist(flat_list[jj], density = False, bins = bins, alpha = 0.6)
        
    plt.legend(string)
    plt.show()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    