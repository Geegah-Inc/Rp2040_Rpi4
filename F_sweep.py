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
#savedirname = "/home/geegah/Documents/RaspberryPi_Anuj/fsweeptest50ns_1/" #change this to whatever you want your directory name to be

savedirname ="/media/geegah//Samsung T5/IMAGER_DATA/ANUJ/FSWEEP_P60ns_/"
if not os.path.exists(savedirname):
    os.makedirs(savedirname)

#folder to dump raw .dat files in 
rawdata_echo_dir = savedirname + "rawdata_echo_sample/"
if not os.path.exists(rawdata_echo_dir):
    os.makedirs(rawdata_echo_dir)
    
rawdata_echo_dir_air = savedirname + "rawdata_echo_air/"
if not os.path.exists(rawdata_echo_dir_air):
    os.makedirs(rawdata_echo_dir_air)


rawdata_noecho_dir = savedirname + "rawdata_noecho/"
if not os.path.exists(rawdata_echo_dir):
    os.makedirs(rawdata_echo_dir)
    
img_save_dir = savedirname + "images/"
if not os.path.exists(img_save_dir):
    os.makedirs(img_save_dir)
    
vid_save_dir = savedirname + "videos/"
if not os.path.exists(vid_save_dir):
    os.makedirs(vid_save_dir)

#3 dummy frames 
#%%
def dummy_frames(n =3):
    for jj in range(n):
        frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg) 
            
    print("Captured "+str(n)+" dummy frames")
    return None        
    

#%%
ser = serial.Serial('/dev/ttyACM0', 115200)
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


def change_freq(f = 1853):
        # SEtup the GPIO
    picoDAQ_Lib.setup_GPIO(pinDict_arg = pinDict_Main)
    
    #Create the sPI object for VCO
    # SPI-0, DEV-1, with 
    # Get VCO SPi object
    spi_obj_vco = picoDAQ_Lib.get_spi_vco()
    freq_target_MHz = f
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
    
    return spi_obj_pico, n_bytes_block_arg


spi_obj_pico, n_bytes_block_arg = change_freq(f = 1890)

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
N_baseline_frames = 1

# Add 2 dummy frames at the beginning
frames_I_baseline, frames_Q_baseline , bufferSignature_1D_baseline, missedBlobCount_1D_baseline = picoDAQ_Lib.getFrames_I_and_Q(spi_ob_arg=spi_obj_pico, nFrames = N_baseline_frames+N_dummy_frames)

frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    
#%% REALTIME_IMAGING_TIMING

mode = 1
#select ADC (1 or 2)
adc = 1
#select timing 25 to 425
start_timing = 90
packet = str(str(mode)+'.'+str(start_timing)+'.'+str(adc)+'\n')
print(packet)
    
ser.write(packet.encode())

 #%% Manually reset
# F SWEEP AIR BASELINE
#frequency values in MHz
import time
start_f = 1700
end_f = 1900
del_f = 1
f = np.arange(start_f, end_f, del_f)

time_i = time.time()

dummy_frames()
I_A = []
Q_A = []
Mag_A = []
Phase_A = []
for jj in range(len(f)):
    freq_int = f[jj]
    spi_obj_pico, n_bytes_block_arg = change_freq(f = freq_int)
    #dummy_frames()
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    
    baseline_echo_fname = rawdata_echo_dir_air + "freq"+str(jj)+".dat"
    picoDAQ_Lib.writeFile(baseline_echo_fname, frames_data_nD_baseline)


    F_IMG_I, F_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
    I_AE_VOLTS, Q_AE_VOLTS = picoDAQ_Lib.convertADCToVolts(F_IMG_I, F_IMG_Q)
    MAG_AE  = np.sqrt(np.square(I_AE_VOLTS)+np.square(Q_AE_VOLTS))
    PHASE_AE = np.arctan2(I_AE_VOLTS, Q_AE_VOLTS)
    
    I_A.append(I_AE_VOLTS)
    Q_A.append(Q_AE_VOLTS)
    Mag_A.append(MAG_AE)
    Phase_A.append(PHASE_AE)
   
    time.sleep(0.1)
    print("DONE frame = "+str(jj))
 

time_f = time.time()
del_time = time_f - time_i
fps = len(f)/del_time
print("average fps = " + str(fps))
print("DONE ACQUIRING BASELINE I AND Q ECHO")

#%%SAMPL:E ACQUISITION
#frequency values in MHz

time_i = time.time()

dummy_frames()
I_S = []
Q_S = []
Mag_S = []
Phase_S= []
for jj in range(len(f)):
    freq_int = f[jj]
    spi_obj_pico, n_bytes_block_arg = change_freq(f = freq_int)
    #dummy_frames()
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)
    
    baseline_echo_fname = rawdata_echo_dir  + "freq"+str(jj)+".dat"
    picoDAQ_Lib.writeFile(baseline_echo_fname, frames_data_nD_baseline)


    F_IMG_I, F_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
    I_SE_VOLTS, Q_SE_VOLTS = picoDAQ_Lib.convertADCToVolts(F_IMG_I, F_IMG_Q)
    MAG_SE  = np.sqrt(np.square(I_SE_VOLTS)+np.square(Q_SE_VOLTS))
    PHASE_SE = np.arctan2(I_SE_VOLTS, Q_SE_VOLTS)
    
    I_S.append(I_SE_VOLTS)
    Q_S.append(Q_SE_VOLTS)
    Mag_S.append(MAG_SE)
    Phase_S.append(PHASE_SE)
   
    time.sleep(0.1)
    print("DONE frame = "+str(jj))
 

time_f = time.time()
del_time = time_f - time_i
fps = len(f)/del_time
print("average fps = " + str(fps))
print("DONE ACQUIRING BASELINE I AND Q ECHO")
#%%

Mag  = np.array(Mag_S) - np.array(Mag_A)
#%%

#NEWFOLDER

folder = "Q OFF 3"
img_save_dir_1 = savedirname + folder+"/"
if not os.path.exists(img_save_dir_1):
    os.makedirs(img_save_dir_1)

fig2,ax2 = plt.subplots(1)
fig2.set_size_inches(4,4)

mytitle = fig2.suptitle(folder + " : ")
im2 = np.flipud(np.rot90(Mag[0],1))
pos201 = ax2.imshow(im2, vmin = -0.1, vmax = 0.1, interpolation = 'bilinear')
fig2.colorbar(pos201)
base_title =folder + " : "
#time how long this takes125
t1 = time.time()
mat_to_plot = mat_off
for cf in range(len(Mag)):
    #read an image
    mytitle.set_text(base_title+"TIME: "+str(start+cf*5)+' ns')
    sub_image = np.flipud(np.rot90(mat_to_plot[cf],1))
    pos201.set_data(sub_image)
    #pos201.set_clim(sub_image.min(), sub_image.max())
    pos201.set_clim(-0.006,0.006)
    plt.pause(0.001)
    #redraw
    fig2.canvas.draw_idle() 
    fig2.savefig(img_save_dir_1+'plot'+str(cf)+'.png')

#%%
mat = Mag_A
median = []
pix = []
for jj in mat:
    median.append(np.median(jj))
    pix.append(jj[60,60])
    
plt.plot(f,median)
plt.title("Magnitude (V) vs frequency: 60ns, Acq:90 after RX")
plt.xlabel("Frequency(MHz)")
plt.ylabel("Magnitude(V)")


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
plt.close("all")
spi_obj_vco.close()  #Close the SPI object
spi_obj_pico.close() # Close the SPI object
spi_obj_dac.close()
GPIO.setwarnings(True)
GPIO.cleanup()
                                          
print("Done")
    