"""Render a session's video as the helmet would see it: the model in BlinkWeights.h run on a FIXED crop at the
session's stored eye position (metadata roi_x/roi_y), with BLINK flashes, the score against the threshold, a running
blink rate, and the hand labels for comparison. Writes <session>/sim_devicelike_model.mp4.

Usage (from the repo root): python fatigue-helmet/python/make_sim_video.py <scratch_dir> [session_number]
"""
import sys,os,re,csv,subprocess,cv2,numpy as np
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
import train_blink_classifier as T
SP=sys.argv[1]; S=sys.argv[2] if len(sys.argv)>2 else '122'; d=f'sessions/session_{S}'

# the model that is on the helmet
t=open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','firmware','src','BlinkWeights.h')).read()
w=np.array([float(x) for x in re.findall(r'(-?\d[\d.eE+-]*)f',t.split('WEIGHTS[]')[1])],dtype=np.float32)
b=float(re.search(r'BIAS\s*=\s*([-\d.eE+]+)f',t).group(1)); th=float(re.search(r'THRESHOLD\s*=\s*([-\d.eE+]+)f',t).group(1))
T.write_model(SP+'/simm.bin',w,b,th)
# device-like: ROI pinned at the stored tap from this session's metadata (no tracking, no drift correction)
meta=dict(l.strip().split('=',1) for l in open(d+'/metadata.txt') if '=' in l)
cx,cy=int(meta['roi_x'])+int(meta['roi_size'])//2,int(meta['roi_y'])+int(meta['roi_size'])//2
r=subprocess.run([T.REPLAY,d+'/frames',SP+'/sim_replay.csv',f'--roi={cx},{cy}','--hog-model='+SP+'/simm.bin','--hog-csv='+SP+'/sim_hog.csv'],capture_output=True,text=True); assert r.returncode==0,r.stderr
hog={int(x['timestamp_ms']):(float(x['score']),int(x['blink'])) for x in csv.DictReader(open(SP+'/sim_hog.csv'))}
lab={int(x['timestamp_ms']):x['state'] for x in csv.DictReader(open(d+'/blink_labels.csv'))}
f=sorted(os.listdir(d+'/frames'),key=lambda p:int(p[:-4])); ts=[int(x[:-4]) for x in f]
# ground-truth blink starts (first frame of each closed run)
gt=set(); prev=False
for t_ in ts:
    c=lab[t_]=='closed'
    if c and not prev: gt.add(t_)
    prev=c
SC=2; W,H=320*SC,240*SC
out=cv2.VideoWriter(d+'/sim_devicelike_model.mp4',cv2.VideoWriter_fourcc(*'mp4v'),10,(W,H+60))
nm=ng=0; flash=0; t0=ts[0]
for i,(name,t_) in enumerate(zip(f,ts)):
    im=cv2.resize(cv2.imread(f'{d}/frames/{name}'),(W,H),interpolation=cv2.INTER_LINEAR)
    sc,ev=hog.get(t_,(None,0))
    if ev: nm+=1; flash=4
    if t_ in gt: ng+=1
    x0,y0=(cx-45)*SC,(cy-90)*SC; cv2.rectangle(im,(x0,y0),((cx+45)*SC,(cy+90)*SC),(0,220,255) if not flash else (60,60,255),2)
    if flash: cv2.rectangle(im,(0,0),(W-1,H-1),(60,60,255),8); cv2.putText(im,'BLINK',(W-150,40),0,1.1,(60,60,255),3,cv2.LINE_AA); flash-=1
    if lab[t_]=='closed': cv2.putText(im,'hand label: CLOSED',(8,H-12),0,0.6,(80,255,80),2,cv2.LINE_AA)
    elif lab[t_]=='unsure': cv2.putText(im,'hand label: unsure',(8,H-12),0,0.6,(200,200,200),1,cv2.LINE_AA)
    bar=np.zeros((60,W,3),np.uint8)
    if sc is not None:
        x=lambda v:int(np.clip((v+4)/8,0,1)*(W-20))+10
        cv2.line(bar,(10,44),(W-10,44),(90,90,90),2); cv2.line(bar,(x(th),34),(x(th),54),(60,60,255),2)
        cv2.circle(bar,(x(sc),44),6,(0,255,0) if sc<=th else (60,60,255),-1)
    m=(t_-t0)/60000
    cv2.putText(bar,f'{int((t_-t0)/1000)//60:02d}:{int((t_-t0)/1000)%60:02d}   model blinks {nm} ({nm/max(m,0.05):.1f}/min)   hand-labelled {ng} ({ng/max(m,0.05):.1f}/min)',(10,20),0,0.5,(255,255,255),1,cv2.LINE_AA)
    out.write(np.vstack([im,bar]))
out.release()
print('device-like ROI centre',cx,cy,'| model events',nm,'| hand-labelled blinks',ng,'| frames',len(f))
