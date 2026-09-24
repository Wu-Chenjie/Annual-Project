import hashlib,json,subprocess,sys
from pathlib import Path

video=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
probe=json.loads(subprocess.check_output(['ffprobe','-v','error','-show_format','-show_streams','-of','json',str(video)]))
duration=float(probe['format']['duration']);stream=probe['streams'][0]
decode=subprocess.run(['ffmpeg','-v','error','-threads','2','-i',str(video),'-f','null','-'],capture_output=True,text=True)
(out/'full-decode.log').write_text(decode.stderr)
assert decode.returncode==0 and not decode.stderr, decode.stderr
assert (stream['width'],stream['height'])==(1920,1080) and stream['codec_name']=='h264'
frames={}
for name,t in [('early',5.),('middle',duration/2),('last',duration-.1)]:
    path=out/(name+'-frame.png')
    subprocess.run(['ffmpeg','-y','-v','error','-ss',str(t),'-i',str(video),'-frames:v','1',str(path)],check=True)
    frames[path.name]=dict(video_time_s=t,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
result=dict(passed=True,full_decode=True,codec=stream['codec_name'],width=stream['width'],height=stream['height'],
    frame_rate=stream['r_frame_rate'],frames=stream.get('nb_frames'),duration_s=duration,bytes=video.stat().st_size,
    sha256=hashlib.sha256(video.read_bytes()).hexdigest(),inspection_frames=frames,
    playback='24x native X11 wall-clock recording; mission metrics use the separate Gazebo simulation clock.')
(out/'media-audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
