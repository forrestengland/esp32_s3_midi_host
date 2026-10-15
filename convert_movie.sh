ffmpeg -i in.mp4 -c copy -bsf:v h264_metadata=colour_primaries=1:transfer_characteristics=1:matrix_coefficients=1 fixed.mp4
ffmpeg -i fixed.mp4 \
  -vf "setparams=colorspace=bt709:color_primaries=bt709:color_trc=bt709,fps=15,scale=320:240:force_original_aspect_ratio=decrease,pad=320:240:(ow-iw)/2:(oh-ih)/2" \
  -pix_fmt yuvj420p -q:v 7 -an -f mjpeg video.mjpeg
