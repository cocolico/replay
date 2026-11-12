from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from render_mount_preview import stl,reduce_mesh,add_mesh,FONT
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
root=Path(__file__).resolve().parents[3]
v,f=reduce_mesh(stl(root/'adaptation_resources/sources/aubo_description/meshes/aubo_i7H/collision/link6.STL'),.0015)
fig=plt.figure(figsize=(11,5.3),facecolor='#f7f8fa')
fig.suptitle('Aubo i7H 末端法兰坐标系（当前 TCP 偏置为零）',fontproperties=FONT,fontsize=16,y=.97)
ax=fig.add_subplot(121,projection='3d');ax.set_facecolor('#f7f8fa')
add_mesh(ax,[{'name':'flange','v':v,'f':f}],color='#aab5c0')
colors=['#d34b45','#279753','#3478d2']
for i,(label,color) in enumerate(zip(['+X','+Y','+Z'],colors)):
 end=np.eye(3)[i]*.075
 ax.quiver(0,0,0,*end,color=color,linewidth=2.8,arrow_length_ratio=.12)
 ax.text(*(end*1.1),label,color=color,fontsize=14,weight='bold')
ax.scatter([0],[0],[0],c='#29323c',s=22)
ax.text(.003,-.014,-.003,'O',fontsize=12,color='#29323c')
ax.set_xlim(-.065,.09);ax.set_ylim(-.065,.09);ax.set_zlim(-.065,.09);ax.set_box_aspect([1,1,1]);ax.view_init(elev=23,azim=-55);ax.dist=7;ax.set_axis_off()
ax.set_title('局部三维视图｜只显示末端',fontproperties=FONT,fontsize=12,pad=3)
ax=fig.add_subplot(122);ax.set_facecolor('#f7f8fa');ax.set_aspect('equal');ax.set_axis_off()
ax.add_patch(Circle((0,0),.0375,facecolor='#d5dce2',edgecolor='#8d9ba8',linewidth=1.5))
for xy,label,color in [((.072,0),'+X',colors[0]),((0,.072),'+Y',colors[1])]:
 ax.annotate('',xy=xy,xytext=(0,0),arrowprops={'arrowstyle':'-|>','lw':2.7,'color':color})
 ax.text(xy[0]+(.005 if xy[0] else 0),xy[1]+(.004 if xy[1] else -.002),label,color=color,fontsize=14,weight='bold',ha='left' if xy[0] else 'center')
ax.add_patch(Circle((0,0),.004,fill=False,edgecolor=colors[2],linewidth=2));ax.scatter([0],[0],s=15,c=colors[2])
ax.text(-.008,-.009,'O',fontsize=12,ha='right')
ax.text(0,-.058,'⊙ +Z 朝向你／朝向连接的手',fontproperties=FONT,fontsize=12,color=colors[2],ha='center')
ax.set_xlim(-.068,.095);ax.set_ylim(-.072,.09)
ax.set_title('从手侧看向法兰端面（沿 −Z 看）',fontproperties=FONT,fontsize=12)
fig.text(.5,.045,'O：法兰安装端面的中心    X/Y：端面内    Z：垂直端面，朝手部    正转：遵循右手定则',fontproperties=FONT,fontsize=11,ha='center')
fig.subplots_adjust(top=.82,bottom=.16,left=.015,right=.96,wspace=.17)
fig.savefig(Path(__file__).parent/'flange-coordinate-frame.png',dpi=175,bbox_inches='tight')
