#!/bin/bash
/usr/src/tensorrt/bin/trtexec --onnx=./model-9/pointpillar.onnx --fp16 --plugins=build/libpointpillar_core.so --saveEngine=./model-9/pointpillar.plan --inputIOFormats=fp16:chw,int32:chw,int32:chw --verbose --dumpLayerInfo --dumpProfile --separateProfileRun --profilingVerbosity=detailed > model-9/pointpillar.8611.log 2>&1
