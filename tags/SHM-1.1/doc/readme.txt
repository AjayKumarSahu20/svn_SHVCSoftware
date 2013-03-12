TAppEncoder -c cfg/encoder_randomaccess_main.cfg -c cfg/per-sequence-svc/BasketballDrive-2x.cfg -q0 22 -q1 22 -b str/BasketballDrive.bin -o0 rec/BasketballDrive_l0_rec.yuv -o1 rec/BasketballDrive_l1_rec.yuv

TAppDecoder -b str/BasketballDrive.bin -ls 2 -o0 rec/BasketballDrive_l0_drec.yuv -o1 rec/BasketballDrive_l1_drec.yuv

For AVC_BASE tests the following additionally should be used.

In the encoder config file for the layer0 ONLY new line should be added. Strictly after InputFile. InputBLFile indicates the path to the BL yuv. Example:
InputFile                     : O:\BasketballDrive_1280x720_50_zerophase_0.9pi.yuv
InputBLFile                 : O:\BasketballDrive_l0_rec.yuv

For decoder, new command line parameters additionally should be used as following
-ibl “BLrecon.yuv” –wdt BLwidth –hgt BLheight
