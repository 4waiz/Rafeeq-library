import subprocess, re
out=subprocess.run(['gz','model','--list'],capture_output=True,text=True,timeout=30).stdout
books=sorted(re.findall(r'book_col_\d+_row_\d+_\w+', out))
def pose(m):
    o=subprocess.run(['gz','model','-m',m,'-p'],capture_output=True,text=True,timeout=25).stdout
    for g in re.findall(r'\[([^\[\]]*)\]',o):
        try: v=[float(x) for x in g.replace(',',' ').split()]
        except ValueError: continue
        if len(v)==3: return v
print('column 4 books (competition row = model row - 1):')
for b in books:
    if '_col_4_' in b:
        p=pose(b)
        m=re.match(r'book_col_(\d+)_row_(\d+)_(\w+)',b)
        print('  %-28s comp_row=%s  world y=%+.4f z=%.4f'%(b,int(m.group(2))-1,p[1],p[2]))
