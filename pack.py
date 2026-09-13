import os,struct,math,pathlib

# 印刷用 STL の入出力ルート。
# OS 依存の絶対パスを埋め込まない (Ubuntu/Windows 双方から実行できるように)。
# 既定は「このスクリプトの隣の ATN_print/」。ATN_PRINT_DIR で上書きできる。
R = str(pathlib.Path(os.environ.get('ATN_PRINT_DIR',
                                    pathlib.Path(__file__).resolve().parent / 'ATN_print')).resolve())
S = R + '/_individual_parts'
RX={'motor_bracket':90.0,'main_plate':180.0}
def rxo(n):
    for k,v in RX.items():
        if n.startswith(k): return v
    return 0.0
def rd(p):
    f=open(p,'rb');d=f.read();f.close()
    n=struct.unpack('<I',d[80:84])[0]
    return n,d[84:84+n*50]
def rot(b,n,rx,rz):
    c=math.cos(math.radians(rx)); s=math.sin(math.radians(rx))
    o=bytearray(); lo=[1e9]*3; hi=[-1e9]*3
    for i in range(n):
        v=list(struct.unpack('<12f',b[i*50:i*50+48])); nv=[]
        for k in range(4):
            x,y,z=v[k*3],v[k*3+1],v[k*3+2]
            y2=y*c-z*s; z2=y*s+z*c
            if rz: x,y2=-y2,x
            nv+=[x,y2,z2]
            if k>0:
                for a,cc in enumerate((x,y2,z2)):
                    if cc<lo[a]: lo[a]=cc
                    if cc>hi[a]: hi[a]=cc
        o+=struct.pack('<12f',*nv)+bytes(2)
    return bytes(o),lo,hi
def tr(t,n,tx,ty,tz):
    o=bytearray()
    for i in range(n):
        v=list(struct.unpack('<12f',t[i*50:i*50+48])); nv=v[0:3]
        for k in range(1,4): nv+=[v[k*3]+tx,v[k*3+1]+ty,v[k*3+2]+tz]
        o+=struct.pack('<12f',*nv)+bytes(2)
    return bytes(o)
C={}
def prep(nm,rz):
    if (nm,rz) not in C:
        n,b=rd(S+'/'+nm+'.stl'); t,lo,hi=rot(b,n,rxo(nm),rz)
        C[(nm,rz)]=(n,t,lo,hi)
    return C[(nm,rz)]
def build(j,fo,it,bed):
    T=bytearray(); tot=0; bx=[]
    for nm,x,y,rz in it:
        n,t,lo,hi=prep(nm,rz)
        T+=tr(t,n,x-lo[0],y-lo[1],-lo[2]); tot+=n
        bx.append((nm,x,y,hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]))
    ov=[]
    for a in range(len(bx)):
        for b in range(a+1,len(bx)):
            p,q=bx[a],bx[b]
            if p[1]<q[1]+q[3] and q[1]<p[1]+p[3] and p[2]<q[2]+q[4] and q[2]<p[2]+p[4]: ov.append((p[0],q[0]))
    mx=max(v[1]+v[3] for v in bx); my=max(v[2]+v[4] for v in bx); mz=max(v[5] for v in bx)
    ok=(not ov) and mx<=bed and my<=bed and mz<=bed; sz=0
    if ok:
        p=R+'/'+fo+'/'+j+'.stl'; f=open(p,'wb')
        f.write(('ATN '+j).encode().ljust(80,b' ')); f.write(struct.pack('<I',tot)); f.write(T); f.close()
        sz=os.path.getsize(p)/1024
    print(('OK ' if ok else '!! ')+j+' %dpt %.0fx%.0fx%.0f %dtri %.0fKB '%(len(it),mx,my,mz,tot,sz)+str(ov))
BARS=['splice_main_%d'%i for i in range(6)]+['splice_bot_%d'%i for i in range(8)]
def dimsof(nm,rz):
    n,t,lo,hi=prep(nm,rz)
    return hi[0]-lo[0],hi[1]-lo[1]
def shelfpack(names,bed,mg,gp):
    it=[]
    for nm in names:
        w,h=dimsof(nm,False)
        rz=False
        if w<h: w,h=dimsof(nm,True); rz=True
        it.append([nm,w,h,rz])
    it.sort(key=lambda p:-p[2])
    out=[]; y=mg; row=[]; rowh=0; x=mg
    for p in it:
        if x+p[1]>bed-mg:
            y+=rowh+gp; x=mg; rowh=0
        out.append((p[0],x,y,p[3]))
        x+=p[1]+gp
        if p[2]>rowh: rowh=p[2]
    return out,y+rowh
def main():
    small=['camera_mast','camera_wedge_oak','minipc_cradle','lidar_mast',
           'motor_bracket_FL','motor_bracket_FR','motor_bracket_RL','motor_bracket_RR']
    lay,ht=shelfpack(small,256.0,4.0,4.0)
    print('big smalls height',ht)
    nb=0
    while nb<len(BARS):
        cand=small+BARS[:nb+1]
        l2,h2=shelfpack(cand,256.0,4.0,4.0)
        if h2>252.0: break
        lay,ht=l2,h2; nb+=1
    print('bars on S1:',nb,'height',ht)
    build('A1_S1_smallparts','A1_256',lay,256.0)
    rest=BARS[nb:]
    qs=['bottom_plate_FL','bottom_plate_FR','bottom_plate_RL','bottom_plate_RR']
    per=(len(rest)+3)//4
    for i,q in enumerate(qs):
        it=[(q,4,4,False)]
        for jj,nm in enumerate(rest[i*per:(i+1)*per]):
            n,t,lo,hi=prep(nm,False)
            rz=(hi[0]-lo[0])>(hi[1]-lo[1])
            it.append((nm,212,4+jj*76,rz))
        build('A1_%d_%s'%(i+1,q.split('_')[-1]),'A1_256',it,256.0)
    for i,q in enumerate(['main_plate_FL','main_plate_FR','main_plate_RL','main_plate_RR']):
        build('K1_%d_%s'%(i+1,q.split('_')[-1]),'K1Max_300',[(q,10,10,False)],300.0)
    build('K1_5_dustbin','K1Max_300',[('dustbin',10,10,False)],300.0)

if __name__ == '__main__':
    main()
