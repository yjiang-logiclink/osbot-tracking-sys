rpicam-vid -t 0 --inline --listen -o tcp://0.0.0.0:8000 --width 2312 --height 1736 --codec mjpeg 
rpicam-vid -t 0 --inline --listen -o tcp://0.0.0.0:8000 --width 1920 --height 1080 --codec mjpeg 
rpicam-vid -t 0 --inline --listen -o tcp://0.0.0.0:8000 --width 3840 --height 2160 --codec mjpeg 
rpicam-vid -t 0 --inline --listen -o tcp://0.0.0.0:8000 --width 1960 --height 1280 --codec mjpeg --lens-position 1.5 --autofocus-mode manual --shutter 800

rpicam-vid -t 0 --inline --listen -o tcp://0.0.0.0:8000 --width 3840 --height 2160 --codec libav --libav-format avi 


rpicam-vid -t 0 --inline -o udp://192.168.1.108:8000 --width 1920 --height 1080 --codec mjpeg

rpicam-vid -t 0 --inline -o udp://192.168.1.108:8000 --width 4624 --height 3472 --codec mjpeg

    SRGGB10_CSI2P,1280x720/0 - Score: 3000
    SRGGB10_CSI2P,1920x1080/0 - Score: 1000
    SRGGB10_CSI2P,2312x1736/0 - Score: 2599.94
    SRGGB10_CSI2P,3840x2160/0 - Score: 1750
    SRGGB10_CSI2P,4624x3472/0 - Score: 3611.94
    SRGGB10_CSI2P,8000x6000/0 - Score: 5083.33
    SRGGB10_CSI2P,9152x6944/0 - Score: 5653.42


0 : arducam_64mp [9248x6944 10-bit RGGB] (/base/axi/pcie@1000120000/rp1/i2c@88000/arducam_64mp@1a)
    Modes: 'SRGGB10_CSI2P' : 1280x720 [120.09 fps - (2064, 2032)/5120x2880 crop]
                             1920x1080 [60.04 fps - (784, 1312)/7680x4320 crop]
                             2312x1736 [30.00 fps - (0, 0)/9248x6944 crop]
                             3840x2160 [20.00 fps - (784, 1312)/7680x4320 crop]
                             4624x3472 [10.00 fps - (0, 0)/9248x6944 crop]
                             8000x6000 [3.00 fps - (624, 472)/9248x6944 crop]
                             9152x6944 [2.70 fps - (0, 0)/9248x6944 crop]


    rpicam-vid -t 0 --inline -o udp://192.168.1.108:8000 --mode 4624:3472:SRGGB10_CSI2P --framerate 20 --codec mjpeg --shutter 10000


rpicam-vid -t 0 --inline   --mode 3840:2160:SRGGB10_CSI2P   --framerate 30   --codec mjpeg  --width 3840 --height 2160 -o udp://192.168.1.105:8000 --shutter 10000
rpicam-vid -t 0 --inline   --mode 1920:1080:SRGGB10_CSI2P   --framerate 30   --codec mjpeg  --width 1920 --height 1080 -o udp://192.168.1.105:8000 --shutter 10000

rpicam-vid -t 0 --inline   --mode 3840:2160:SRGGB10_CSI2P   --framerate 30 --width 3840 --height 2160 -o udp://192.168.0.118:8000 --shutter 10000 --codec libav --libav-format avi 

rpicam-vid -t 0 --inline   --mode 4624:3472:SRGGB10_CSI2P   --framerate 20   --codec mjpeg  --width 4624 --height 3472 -o udp://192.168.0.118:8000 --shutter 10000
rpicam-vid -t 0 --inline   --mode 4624:3472:SRGGB10_CSI2P   --framerate 20   --codec mjpeg  --width 4624 --height 3472 -o udp://192.168.0.118:8000 --shutter 10000



rpicam-vid -t 0 --inline   --mode 3840:2160:SRGGB10_CSI2P   --framerate 30   --codec mjpeg  --width 3840 --height 2160 -o udp://192.168.1.44:8000 --shutter 500
rpicam-vid -t 0 --inline   --mode 3840:2160:SRGGB10_CSI2P   --framerate 30   --codec mjpeg  --width 3840 --height 2160 -o udp://127.0.0.1:8000 --shutter 500


sshpass -p 'osbot' rsync -av -e ssh ./ cam@osbot-pi5-1:~/osbot-tracker/osbot-tracking-sys/ --exclude '.*'