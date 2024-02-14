"""
Spyder Editor

This is a temporary script file.
"""
#import spidev
#import RPi.GPIO as GPIO
import time
#import spidev
import picoDAQ_Lib   # custom functions for DataAcquisition
import Utils_SA_pico # custom functions for generic tasks
import RPi.GPIO as GPIO
import matplotlib.pyplot as plt
import os
import datetime
import numpy as np
#%%
# For saving later
scriptTime = datetime.datetime.now()
strTimeFolder = datetime.datetime.strftime(scriptTime, '%Y%m%d')

GPIO_PINNO_TEST = 27

# VCO, ad4351 PINS, SPI0, DEV0
GPIO_NUM_VCO_LE = 7  #SPI0_CE1 from Rpi
#GPIO_NUM_VCO_CE = 8 # Note that SPI GPIO8 is connected to VCO-LE and VCO CE-is floating, since it is pulled high all the time on the board
#GPIO_NUM_VCO_CLK = 11
#GPIO_NUM_VCO_DATA = 10

## Raspberry PI, RP PICO, SPI1, DEV0
#GPIO_NUM_PICO_CE = 18 # GPIO 28 @ PIN12
#GPIO_NUM_PICO_CLK = 21 # GPIO 21 @ PIN40
#GPIO_NUM_PICO_MISO = 19  # GPIO 19 @ PIN35

# SPI DAC new device, Configured on SPI0 with dedicated GPIO for C#
GPIO_NUM_DAC_MOSI = 10
GPIO_NUM_DAC_MISO = 9
GPIO_NUM_DAC_CLK = 11
GPIO_NUM_DAC_CE = 8 # STANDARD SPI0, dev0 CE, not used for DAC
GPIO_NUM_DAC_CE0B =22 # Dedicated DAC for 


##GPIO assignments;
#gpDacMosi=10
#gpDacMiso=9
#gpDacSck=11
#gpDacCe0=8
#gpTest=25 
#gpDacCe0b = 22
#gpTest = 27
#
#gpVcoLe = 7
#gpVCOCE=8
#gpVCOCLK=11
#gpVCODATA=10

# Connections
# RPI           Target
# Pin15         CE ----> DAC----> right connector
# Pin19         Data ----> DAC----> right connector
# Pin21         Data ----> PICO----> PICO connector
# Pin23         CK ----> DAC & CK-----PICO : ----> shorted externally
# Pin24         CS ----> PICO----> PICO connector
# Pin26         LE ----> VCO----> Right connector


pinDict_Main = dict(gpio_num_PINNO_TEST = GPIO_PINNO_TEST, 
                    gpio_num_DAC_CE0B = GPIO_NUM_DAC_CE0B,
                    gpio_num_VCO_LE = GPIO_NUM_VCO_LE);
                    
                    
                    

# CLKn frequency for SPI in Hz
CLK_FREQ_SPI_PICO =int(1e6)



# SEtup the GPIO
picoDAQ_Lib.setup_GPIO(pinDict_arg = pinDict_Main)

#print("Testing GPIO_NUM_VCO_LE")
#picoDAQ_Lib.gpio_testroutine_with_testpin(GPIO_NUM_DAC_MOSI, flag_include_delay=True)
#%%
#Create the sPI object for VCO
# SPI-0, DEV-1, with 
# Get VCO SPi object
spi_obj_vco = picoDAQ_Lib.get_spi_vco()


# VCO settings and registers
freq_target_MHz = 1853.5
OUTEN_user = 1
PSET_user =3
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



#%% DAC update


# Setup SPI for DAC MAXIM 5252, Bus0, DEV 0 but with dedicated CS GPIO pin at GPIO_NUM_DAC_CE0B

DAC_val = 3815
spi_obj_dac = picoDAQ_Lib.get_spi_dac_MAXIM5123()
picoDAQ_Lib.writeDAC_MAXIM5123(spi_obj_dac, DAC_val, GPIO_NUM_DAC_CE0B)
#spi_obj_dac.close()

#%%
# GET PICO SPI object
spi_obj_pico = picoDAQ_Lib.get_spi_pico(CLK_FREQ_SPI_PICO)


# %% Set Directories

# number of dummy frames
N_dummy_frames=2

# Flag to SAve figures
flagSaveFig = True
# flag to Save Data
flagSaveData = True
# Create the folder if necessary
dataSaveDirParent = "/home/geegah/Documents/RaspberryPi_Anuj/DATA"



dataSaveDir = os.path.join(dataSaveDirParent,"Processed_"+strTimeFolder)
if not os.path.isdir(dataSaveDir):
    print("Created the folder:", dataSaveDir)
    os.makedirs(dataSaveDir)

# Create Sub folders for rawdata/ image video csv
imageSave_SubDirPath = os.path.join(dataSaveDir, "images")
rawDataSave_SubDirPath = os.path.join(dataSaveDir,"rawdata")
videoSave_SubDirPath = os.path.join(dataSaveDir,"video")

# Create the new directory if they do not exist
newDirPath_List = [imageSave_SubDirPath, rawDataSave_SubDirPath, videoSave_SubDirPath]
for curDirPath in newDirPath_List:
    Utils_SA_pico.createDirectoryIfNotExist(newSaveDir=curDirPath)

# Create a list of directories to be saved
saveDirList_Dict = dict(parent=dataSaveDirParent,
                        dataMainSaveDir=dataSaveDir,
                        imageSaveDir = imageSave_SubDirPath,
                        rawdataSaveDir = rawDataSave_SubDirPath,
                        videoSaveDir = videoSave_SubDirPath)

print("Directories are setup!")

#%% Do A Dummy Read

# Number of Frames
N_baseline_frames = 0

# Add 2 dummy frames at the beginning
frames_I_baseline, frames_Q_baseline , bufferSignature_1D_baseline, missedBlobCount_1D_baseline = picoDAQ_Lib.getFrames_I_and_Q(spi_ob_arg=spi_obj_pico, nFrames = N_baseline_frames+N_dummy_frames)


#%% Manually reset
print("\n\n\nMake sure you manually reset RP2040 NOW")
print("Likely because there is no CS, startup data is not well synchronized until RESET")
a=input("Press Enter to continue after manual reset")


# Save the Baseline Frames

# Block read Size
n_bytes_block_arg=1<<12

# Number of Frames
N_baseline_frames = 2

## Add 2 dummy frames at the beginning
#frames_I_baseline, frames_Q_baseline , bufferSignature_1D_baseline, missedBlobCount_1D_baseline = picoDAQ_Lib.getFrames_I_and_Q(spi_ob_arg=spi_obj_pico, nFrames = N_baseline_frames+N_dummy_frames)
#

frames_data_nD_baseline,timeStamp_1D_baseline, flagSignatureFound_1D_baseline, bufferSignature_1D_baseline, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state(spi_obj_arg = spi_obj_pico,nFrames = N_baseline_frames+N_dummy_frames,\
                                                                       n_bytes_block_arg=n_bytes_block_arg)



# Save format
#frames_baseline_avg
# DIM0: Number of frames, size: N_frames
# DIM1: I or Q, 0:I, 1:Q size:2
# DIM2: N_rows
# DIM3: N_cols 


## Takethe mean of the baseline frames and save
#frames_I_baseline_avg = (np.mean(frames_I_baseline[-N_baseline_frames:,...], axis=0)).astype('uint16')
#frames_Q_baseline_avg = (np.mean(frames_Q_baseline[-N_baseline_frames:,...], axis=0)).astype('uint16')

frames_baseline_avg = np.reshape(np.mean(frames_data_nD_baseline[-N_baseline_frames:,...], axis=0).astype('uint16'), (1,2,picoDAQ_Lib.N_rows, picoDAQ_Lib.N_cols))

baseline_file_name = "baseline.npy"
baseline_file_path = os.path.join(saveDirList_Dict["rawdataSaveDir"], baseline_file_name)

# Save file
if flagSaveData:
    np.save(baseline_file_path, frames_baseline_avg)
    print("Baseline data saved to %s"%(baseline_file_path))

# close all the figures
plt.close("all")

# plot Baseline Figure

# Turn the interactive mode on
# https://www.geeksforgeeks.org/how-to-update-a-plot-in-matplotlib/
plt.ion()
fig02, ax02, h_plot_list_02, h_colorbar_list_02 = picoDAQ_Lib.plot_I_Q_Mag_imageFrame(frames_baseline_avg, frameIdx = 0, vmin_arg=0, vmax_arg=4000 )

# Get the baseline Average
baseline_I_mean_allpixels =np.mean(frames_baseline_avg[0,0,...])
baseline_Q_mean_allpixels =np.mean(frames_baseline_avg[0,1,...])
h_suptitle = fig02.suptitle("Baseline Data:\n I_Mean: %3.2f Q_Mean: %3.2f"%(baseline_I_mean_allpixels,baseline_Q_mean_allpixels))
# Non blocking show
plt.show(block=False)

##%% Do test for block read
##n_bytes_block_arg=1<<12
#
#nFrames = 10
#block_read_start_time = datetime.datetime.now()
##data_all_bytes, frameDataOut, bufferSignature, missedBlobCount = picoDAQ_Lib.read_block_single_frame_at_unlocked_state(spi_obj_arg= spi_obj_pico, n_bytes_block_arg=n_bytes_block_arg)
#frames_data_nD,timeStamp_1D, flagSignatureFound_1D, bufferSignature_1D, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state(spi_obj_arg = spi_obj_pico,nFrames = nFrames,\
#                                                                       n_bytes_block_arg=n_bytes_block_arg)
#
#
#
#block_read_elapsed_time_sec = (datetime.datetime.now()-block_read_start_time).total_seconds()
#
#print("\n\n\n---Reading with {} bytes per block took {:3.2f}seconds for {}".format(n_bytes_block_arg,block_read_elapsed_time_sec, nFrames))
#print("\n\n\n---Number of frames with missing/wrong signatures: %i in %i "%(np.logical_not(flagSignatureFound_1D).nonzero()[0].size,nFrames))


#%% Multiple Frame Capture
# Number of Frames
N_sample_frames = 5




# Skip this one for Direct plotting
if True:
    
    # Manually reset
    print("\n\n\nMMake sure the sample is there")
    a=input("Press Enter to continue getting sample data\n")
    start_time_saving = datetime.datetime.now()
    
    # Add 2 dummy frames at the beginning
    #data_all_bytes, frameDataOut, bufferSignature, missedBlobCount = picoDAQ_Lib.read_block_single_frame_at_unlocked_state(spi_obj_arg= spi_obj_pico, n_bytes_block_arg=n_bytes_block_arg)
    frames_data_nD,timeStamp_1D, flagSignatureFound_1D, bufferSignature_1D, missedBlobCount_1D = picoDAQ_Lib.read_block_singleOrMultiple_frames_at_unsynchronized_state(spi_obj_arg = spi_obj_pico,nFrames = N_sample_frames,\
                                                                           n_bytes_block_arg=n_bytes_block_arg)


    # subtract the background
    
    frames_delta = frames_data_nD.astype('int32') - frames_baseline_avg.astype('int32')
    
    sample_file_name = "sample.npy"
    sample_file_path = os.path.join(saveDirList_Dict["rawdataSaveDir"], sample_file_name)
    
    # Save file
    if flagSaveData:
        np.save(sample_file_path, frames_delta)
        print("Sample data saved to %s"%(sample_file_path))
        
    elapsed_time_saving_sec = (datetime.datetime.now()-start_time_saving).total_seconds()
    
    print("SAved %i frames in %f seconds"%(N_sample_frames, elapsed_time_saving_sec))


print("\n\n\n---Saving with {} bytes per block took {:3.2f}seconds for {}".format(n_bytes_block_arg,elapsed_time_saving_sec, N_sample_frames))
print("\n\n\n---Number of frames with missing/wrong signatures: %i in %i "%(np.logical_not(flagSignatureFound_1D).nonzero()[0].size,N_sample_frames))




#%% Convert saved data to images
saved_frames_delta = np.load(sample_file_path)

#for frameIdx in range(saved_frames_delta.shape[0]-1, saved_frames_delta.shape[0]):
for frameIdx in range(saved_frames_delta.shape[0]):
    
    fig01, ax01, h_plot_list, h_colorbar_list = picoDAQ_Lib.plot_I_Q_Mag_imageFrame(frames=saved_frames_delta, frameIdx = frameIdx, vmin_arg=-50, vmax_arg=50)
    fig01.suptitle("Frame %i: Elapsed Time for %i frames =%f sec, @CLK=%.0f kHz With Block Read of %i bytes"%(frameIdx, N_sample_frames, elapsed_time_saving_sec,(CLK_FREQ_SPI_PICO*1e-3),n_bytes_block_arg))
    if flagSaveFig:
        figFileName="Frames_delta_frameIdx_%i_CLK_%.0f_kHz_block_%i_bytes.png"%(frameIdx, (CLK_FREQ_SPI_PICO*1e-3),n_bytes_block_arg)
        figFilePath = os.path.join(saveDirList_Dict["imageSaveDir"],figFileName )
                        
        plt.savefig(figFilePath)
        print("Saved frame %i to %s"%(frameIdx,figFilePath))
        
        plt.close("all")

#%% Plot the sample frames


# Uncomment
# For Live Plottting (Uses the old function without block read)
if False:
    print("Live Plotting !")
    frames_sample , bufferSignature_1D, missedBlobCount_1D, fig_live_plot = picoDAQ_Lib.getFrames_I_and_Q_merged_and_LivePlot(\
                                                                                                               spi_ob_arg=spi_obj_pico,
                                                                                                               nFrames = N_sample_frames,
                                                                                                               frames_baseline_avg =frames_baseline_avg,
                                                                                                               flag_livePlot=True,\
                                                                                                               vmin_arg=-50, vmax_arg=50)
    
    


  #%%
plt.close("all")
spi_obj_vco.close()  #Close the SPI object
spi_obj_pico.close() # Close the SPI object
spi_obj_dac.close()
GPIO.setwarnings(True)
GPIO.cleanup()
                                          
print("Done")