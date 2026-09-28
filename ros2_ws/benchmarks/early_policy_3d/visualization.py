#!/usr/bin/env python3
"""Same live sensor/vehicle visualization, with an explicit baseline label."""
import exploration_visualization_node as common
_original=common.Visualization.__init__
class LabelPublisher:
    def __init__(self,pub,node):self.pub=pub;self.node=node
    def publish(self,out):
        for m in out.markers:
            if m.ns=='live_status':
                d=self.node.diag;g=d.get('graph',{}).get('0',{})
                m.text=m.text.replace('FUSED EXPLORATION','EARLY POLICY / 3D ADAPTER')
                lines=m.text.splitlines()
                lines[2]=f"Central full-grid graph: {g.get('nodes',0)} nodes"
                lines[3]='Frontiers > graph Voronoi > pairwise heuristic > common trajectory'
                m.text='\n'.join(lines)
        self.pub.publish(out)
def init(self):
    _original(self);self.pub=LabelPublisher(self.pub,self)
common.Visualization.__init__=init
if __name__=='__main__':common.main()
