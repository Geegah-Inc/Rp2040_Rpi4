#import modules

import serial #for connecting Rpi4 to Rp2040 via serial port
import picoDAQ_Lib   # custom functions for DataAcquisition
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import os
import numpy as np
import time

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
  
#%%OPEN THE MICROCONTROLLER SERIAL PORT
# change the port name accordingly
#use "python -m serial.tools.list_ports" in terminal to view connected ports

ser = serial.Serial('/dev/ttyACM0', 115200) #'/dev/ttyACM0' is unique to RP2040 


def acq_MODE(mode = 1, time = 125):
    #function to switch the timing mode
    #select mode for acquisition:
        #Mode 0 = ECHO AND NO_ECHO TOGGLE
        #Mode 1 = ECHO TIMING # NORMAL IMAGING #
        #Mode 2 = Sweep through the starttime = time, end time = 450 ns,step = 5ns 
    #time is time after TX disable/RX enable in nano seconds(ns)
  
    mode = mode
    adc = 1
    timing = time
    packet = str(str(mode)+'.'+str(timing)+'.'+str(adc)+'\n')
    print(packet)
    ser.write(packet.encode())

#initialize with mode 1 = Echo acquisition
acq_MODE()
  

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


#FRAME ACQUISITIONS
frames = 100
time_i = time.time()
I_echo = []
Q_echo = []

for jj in range(frames):
    frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state_BIN_1Frame(spi_obj_arg = spi_obj_pico,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)

    baseline_echo_fname = savedirname + "baseline_echo"+str(jj)+".dat"
    picoDAQ_Lib.writeFile(baseline_echo_fname, frames_data_nD_baseline)  
    F_IMG_I, F_IMG_Q = picoDAQ_Lib.convertToIQImage(frames_data_nD_baseline)
    I_AE_VOLTS, Q_AE_VOLTS = picoDAQ_Lib.convertADCToVolts(F_IMG_I, F_IMG_Q)
    I_echo.append(I_AE_VOLTS)
    Q_echo.append(Q_AE_VOLTS)
    time.sleep(0.001)
    print("DONE frame = "+str(jj))
  
time_f = time.time()
del_time = time_f - time_i
fps = frames/del_time
print("average fps = " + str(fps))
print("DONE ACQUIRING BASELINE I AND Q ECHO")

#%%CLOSING SPIO, SERIAL, and plots
plt.close("all")
spi_obj_vco.close()  #Close the SPI object
spi_obj_pico.close() # Close the SPI object
spi_obj_dac.close()
GPIO.setwarnings(True)
GPIO.cleanup()                                      
ser.close()
