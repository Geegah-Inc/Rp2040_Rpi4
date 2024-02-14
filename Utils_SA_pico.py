#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan  7 00:36:42 2022

@author: ardanuc
"""
import matplotlib.pyplot as plt
import os
import datetime

def closeAllFigs():
    figList = plt.get_fignums()
    for fignum in figList:
        plt.close(fignum)
        print("Closed figure %i"%fignum)
        
def changeAxesFontSizeAll(ax, fontsize, flagIncludeLegend=False):
    itemList = ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                 ax.get_xticklabels() + ax.get_yticklabels())
    # Add legend texts if desired
    if flagIncludeLegend:
        itemList.extend(list(ax.legend().get_texts()))
    for item in (itemList):
        item.set_fontsize(fontsize)
        

def createDirectoryIfNotExist(newSaveDir=None):
    if newSaveDir is not None:            
        if not os.path.isdir(newSaveDir):
            print("Creating folder:%s"%newSaveDir)
            os.makedirs(newSaveDir)
    else:
        print("No folder created since no input given")


# Create Video from list of images

def createVideo_From_ListOfImagePaths(imagePathList, outSaveFolder=None, outFileName=None, frameRate_FPS=4):
    """
     Save a video from a list of images that are provided through a list of abolsute paths

    Parameters
    ----------
    imagePathList : list of paths
        Absolute Paths .
    outSaveFolder : Absolute Path, optional
        The folder to save the video files. The default is None.
    outFileName : File Name, optional
        Output File Name. The default is None.
    frameRate_FPS : integer, optional
        Video frame rate in FPS. The default is 4.

    Returns
    -------
    None.

    """
    
    
    import cv2
    img_array = []
    nFrames = len(imagePathList)
    
    if nFrames<2:
        print("Not enough frames: %i Frame. Returning without frame cureation"%nFrames)
        return
    
    # initialize to arguments
    _outSaveFolder = outSaveFolder
    _outFileName = outFileName
    
    # If outSaveFolder is not defined assume the same folder as the image files
    if not outSaveFolder:
        _outSaveFolder=os.path.dirname(imagePathList[0])
        print("Output SAve folder is not specified so it is set to first files's parent directory:%s"%outSaveFolder)
    
    if not outFileName:
        dateStr_forFileName = datetime.datetime.strftime(datetime.datetime.now(),"%Y%m%dT%H%M")
        # if the output file name is not set, take the 
        _outFileName = "videoFromImages_%s.avi"%dateStr_forFileName
    
    
    # Make sure the Folder exists
    createDirectoryIfNotExist(_outSaveFolder)
    
    #Final vide PathName
    videoPathFileName = os.path.join(_outSaveFolder,_outFileName)


    # Iterate through the image files
    for curImagePath in imagePathList:
        
        img_cur = cv2.imread(curImagePath)

        height, width, layers = img_cur.shape
        size = (width,height)
        img_array.append(img_cur)


    
    out = cv2.VideoWriter(videoPathFileName,cv2.VideoWriter_fourcc(*'DIVX'), frameRate_FPS, size)
     
    for i in range(len(img_array)):
        out.write(img_array[i])
    out.release()
    
    print("Video file saved to %s"%videoPathFileName)

        
        
        