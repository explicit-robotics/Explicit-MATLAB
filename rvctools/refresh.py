'''
tool to sync working versions with MATLAB drive
rsync not nuanced enough with difference detection, date/size only
PROBLEM is that many files in working folders are temporary, not checked in to git
probably need to clone these copies before doing stuff

'''
from os import listdir
import os.path
import filecmp
import shutil



def ismfile(path1, f):
    f = os.path.join(path1, f)
    return os.path.isfile(f) and os.path.splitext(f)[1] == '.m'

def refresh(src, dst):
    src = os.path.expanduser(src)
    dst = os.path.expanduser(dst)

    # get list of all source files
    files = [f for f in listdir(src) if ismfile(src, f)]

    for file in files:
        update = False
        # create full path to source and destination files
        f1 = os.path.join(src, file)
        f2 = os.path.join(dst, file)

        # check if dst exists, it might be a new file
        if not os.path.exists(f2):
            print 'NEW: ',
            update = True;
        else:
            d = filecmp.cmp(f1, f2, shallow=False)
            if not d:
                print 'UPDATED: ',
                update = True

        if update:
            #shutil.copyfile(f1, f2)
            print 'Copy %s -> %s' % (f1, f2)


refresh('~/code/spatial-math', 'spatial-math')
refresh('~/code/MATLAB-toolboxes/robotics-toolbox-matlab', 'robot')
refresh('~/code/MATLAB-toolboxes/machinevision-toolbox-matlab','vision')
refresh('~/code/MATLAB-toolboxes/toolbox-common-matlab','common')
