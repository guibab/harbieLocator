lstPoints = "-0.5 0.5 -0.5 -p -0.5 0.5 0.5 -p 0.5 0.5 0.5 -p 0.5 0.5 -0.5 -p -0.5 0.5 -0.5 -p -0.5 -0.5 -0.5 -p 0.5 -0.5 -0.5 -p 0.5 0.5 -0.5 -p 0.5 -0.5 -0.5 -p 0.5 -0.5 0.5 -p 0.5 0.5 0.5 -p 0.5 -0.5 0.5 -p -0.5 -0.5 0.5 -p -0.5 0.5 0.5 -p -0.5 -0.5 0.5 -p -0.5 -0.5 -0.5"

myNewArrayofLines = []
for el in lstPoints .split ("-p"):   
    arr = ", ".join ([ el+"f" for el in el.strip().split(" ") ])
    newLine = "{{ {0} }} ".format (arr)
    myNewArrayofLines .append (newLine )

toPrint = "static float listLines[][3] = {{ {0} }}; ".format(",\n".join(myNewArrayofLines     ))        
print toPrint 


#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
from maya import cmds
cmds.loadPlugin (r"C:\Users\guillaume\Documents\DEV\Maya\cpp\harbieLocator\BUILD20165\Debug\harbieLocator.mll")
cmds.createNode ("harbieLocator")
cmds.createNode ("makeHarbieCurve")

#------------------------------------------------------------------------------------------------------------------------
def createNodeCurve () :
    tr,=cmds.circle (ch=False)
    shp,=cmds.listRelatives (tr, s=1)
    mhc= cmds.createNode ("makeHarbieCurve")
    cmds.connectAttr (mhc+".outputCurve", shp+".create")
    return tr
nbLoc = 10
for x in range (nbLoc) : 
    for y in range (nbLoc) : 
        for z in range (nbLoc) : 
            prt=createNodeCurve ()
            cmds.xform (prt, t= [x,y,z])

#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------

cmds.loadPlugin ("harbieLocator")

nbLoc = 10
for x in range (nbLoc) : 
    for y in range (nbLoc) : 
        for z in range (nbLoc) : 
            loc = cmds.createNode ("harbieLocator")                    
            prt,=cmds.listRelatives (loc, p=True)
            cmds.xform (prt, t= [x,y,z])




icon="Arrow"
for x in range (nbLoc) : 
    for y in range (nbLoc) : 
        for z in range (nbLoc) : 
          nd = cmds.createNode ("transform", n=icon )
          tools.objects.createShape(nd, icon, 1.0, True)
          cmds.xform (nd, t= [x,y,z])

#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------

from mrigtools.mrigtools import MRigToolsWindow
from mrigtools import tools
from maya import cmds
icons = dcc.maya.icon.iconList(title=True)
size = 1.
replace = True
for icon in icons :
    nd = cmds.createNode ("transform", n=icon )
    try : 
        tools.objects.createShape(nd, icon, size, replace)
    except : 
        pass


#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
for ind, icon in enumerate (icons) : 
    print 'CHECK_MSTATUS(enumAttr.addField("{1}", {0}));'.format (ind, icon)
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
uncapitalize = lambda s: s[0].lower() + s[1:] if s else ''


def printingFunction ():
  a='''if (displayIndex == {2}) {{ // {0}
  for (int i = 0; i < {0}Count; i++)
    linesPoints.append(MPoint(listLines{1}[i][0], listLines{1}[i][1], listLines{1}[i][2]) * matPreRotate);
  }}
  '''
  sel = cmds.ls (sl=True)
  toPrint = "\n\n"
  for ind, icon in enumerate(sel) :
      if ind !=0 : toPrint+="else "
      toPrint += a.format (uncapitalize(icon), icon.capitalize(), ind)
      toPrint +="\n"
  cmds.warning(toPrint) 

def BBFunction ():
  a='''if (displayIndex == {2}) {{ // {0}
    this->theBoundingBox = MBoundingBox(MPoint({0}BB[0][0], {0}BB[0][1], {0}BB[0][2]), MPoint({0}BB[1][0], {0}BB[1][1], {0}BB[1][2]));
  }}
  '''
  sel = cmds.ls (sl=True)
  toPrint = "\n\n"
  for ind, icon in enumerate(sel) :
      if ind !=0 : toPrint+="else "
      toPrint += a.format (uncapitalize(icon), icon.capitalize(), ind)
      toPrint +="\n"
  cmds.warning(toPrint) 

def enumAttrFunction ():
  sel = cmds.ls (sl=True)
  toPrint = "\n\n"  
  for ind, icon in enumerate (sel) : 
      toPrint+= 'CHECK_MSTATUS(enumAttr.addField("{1}", {0}));'.format (ind, icon)
      toPrint +="\n"
  cmds.warning(toPrint) 


"""
definitionCurve () 
printingFunction ()
BBFunction()
enumAttrFunction ()
"""
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------

from maya import cmds

def definitionCurve () :
  sel = cmds.ls (sl=True)
  toPrint=""
  for curveName in sel:
    varName = "listLines"+curveName.capitalize()
    isOpen = cmds.getAttr (curveName+".form")== 0 
    print      "\n\n"
    pts=  cmds.xform (curveName+".cv[*]", q=True, ws=True, t=True)
    origPts = zip(pts[0::3], pts[1::3], pts[2::3])
    if not isOpen : origPts.append (origPts[0])
    myNewArrayofLines = []

    for thePt in origPts :
       arr = "{0:.3f}f, {1:.3f}f,{2:.3f}f".format (*thePt )
       newLine = "{{ {0} }} ".format (arr)
       myNewArrayofLines .append (newLine )

    lowIcon = uncapitalize(curveName)
    toPrint += "\n\nstatic float {1}[{2}][3] = {{ {0} }}; ".format(",\n".join(myNewArrayofLines     ), varName,len(origPts) )        
    toPrint += "\nstatic int {0}Count = {1};".format (lowIcon, len(origPts))
    toPrint += "\nstatic float {0}BB[2][3] = ".format (lowIcon)
    bb = cmds.exactWorldBoundingBox(curveName)
    toPrint += "{{  {{ {0:.2f}f, {1:.2f}f, {2:.2f}f }} ,{{ {3:.2f}f, {4:.2f}f, {5:.2f}f }} }};\n".format (*bb)

  cmds.warning(toPrint) 
    


#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------

from maya import cmds
sel = cmds.ls (sl=True)
varName = "listLines"
print      "\n\n"
for ind, nd  in enumerate (sel) : 
   pts=  cmds.xform (nd+".cv[*]", q=True, ws=True, t=True)
   origPts = zip(pts[0::3], pts[1::3], pts[2::3])
   myNewArrayofLines = []
   
   for thePt in origPts :
       arr = "{0:.3f}f, {1:.3f}f,{2:.3f}f".format (*thePt )
       newLine = "{{ {0} }} ".format (arr)
       myNewArrayofLines .append (newLine )
   toPrint = "static float {1}{2}[][3] = {{ {0} }}; ".format(",\n".join(myNewArrayofLines     ), varName,ind )        
   print      toPrint   
       

#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------

from maya import cmds, mel
cmds.loadPlugin ("harbieLocator")

nbObj = 10
type = "polyCube"
type = "harbie""
for i in range (nbObj ):
    for j in range (nbObj ):
        for k in range (nbObj) : 
            if type == "harbie":  
                newLoc = cmds.createNode ("harbieLocator")
                cmds.setAttr (newLoc+".display", 1)
                cmds.setAttr (newLoc+".localPosition", i * 1.1, j*1.1, k*1.1)
            else if type = "crv" : 
                cmds.circle( center=[0.0, 0.0, 0.0], normal=[1.0, 0.0, 0.0], sweep=360.0, radius=0.5, degree=1, useTolerance=False, tolerance=0.01, sections=12, constructionHistory=False)
                cmds.move (i * 1.1, j*1.1, k*1.1,a=True)
                cmds.circle( center=[0.0, 0.0, 0.0], normal=[.0, 1.0, 0.0], sweep=360.0, radius=0.5, degree=1, useTolerance=False, tolerance=0.01, sections=12, constructionHistory=False)
                cmds.move (i * 1.1, j*1.1, k*1.1,a=True)
                cmds.circle( center=[0.0, 0.0, 0.0], normal=[.0, 0.0, 1.0], sweep=360.0, radius=0.5, degree=1, useTolerance=False, tolerance=0.01, sections=12, constructionHistory=False)
                cmds.move (i * 1.1, j*1.1, k*1.1,a=True)            
            else : 
                res= mel.eval(type)
                cmds.move (i * 1.1, j*1.1, k*1.1,a=True)
            
#for newLoc in cmds.ls (type = "harbieLocator") : cmds.setAttr (newLoc+".display", 1)