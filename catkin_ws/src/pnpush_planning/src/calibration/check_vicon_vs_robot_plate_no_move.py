#!/usr/bin/env python

# this is to find out the transform between the vicon frame and robot frame

import tf.transformations as tfm
import numpy as np
from ik.roshelper import lookupTransform
from ik.roshelper import ROS_Wait_For_Msg
from ik.helper import *
import tf
import rospy
from tf.broadcaster import TransformBroadcaster
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *

from visualization_msgs.msg import Marker
from marker_helper import createMeshMarker
from vicon_bridge.msg import Markers

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from rigid_transform_3D import rigid_transform_3D
import tf.transformations as tfm
from ik.ik import setSpeed

#limits = [0.32, 0.38, -0.2, +0.1, 0.27, 0.27]  #[xmin, xmax, ymin, ymax, zmin, zmax]
limits = [0.36, 0.42, -0.25, -0.05, 0.30+0.02-0.05, 0.30+0.02]  #[xmin, xmax, ymin, ymax, zmin, zmax]
nseg = [3, 3, 3]
nrotate = 8
ori = [0, 0, 1, 0]
globalacc = 2             # some big number means no limit, in m/s^2

def xyztolist(pos):
    return [pos.x, pos.y, pos.z]

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    setCartRos(*param)

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))

def maxindex(a):
    maxx = a[0]
    maxIndex = 0
    for i in range(len(a)):
        if a[i] > maxx:
            maxx = a[i]
            maxIndex = i
    return maxIndex

viconpts = [[0.02883246,0.02685887,0.07556457],
 [0.028739  ,0.02636022,0.07556215],
 [0.02827735,0.02607249,0.07564677],
 [0.02771523,0.02628212,0.075606  ],
 [0.0275621 ,0.02688295,0.07558303],
 [0.02795363,0.02720962,0.07555364],
 [0.02845669,0.02725467,0.07554586],
 [0.02883767,0.02690186,0.07556651],
 [0.02883528,0.02685837,0.10049953],
 [0.02879448,0.02630394,0.10048569],
 [0.02836304,0.02607282,0.10057371],
 [0.02771442,0.02627986,0.10053713],
 [0.02761153,0.0268325 ,0.10050866],
 [0.02799173,0.02717855,0.10047576],
 [0.02851523,0.02721722,0.1004843 ],
 [0.02886315,0.02687761,0.10050285],
 [0.02912647,0.02664808,0.12547306],
 [0.02901614,0.02613285,0.12544882],
 [0.02856966,0.02588471,0.12554922],
 [0.02791499,0.02614204,0.12551301],
 [0.02786198,0.02662466,0.12545546],
 [0.02819326,0.0270594 ,0.12542704],
 [0.02872405,0.02707866,0.12547256],
 [0.02903114,0.0267776 ,0.12549033],
 [0.02657998,0.12608085,0.0757974 ],
 [0.02632676,0.12568698,0.07581021],
 [0.02576556,0.12560521,0.07585513],
 [0.02529657,0.12597156,0.07586148],
 [0.02534939,0.12646786,0.07585768],
 [0.02571806,0.12677823,0.0757962 ],
 [0.02625745,0.126664  ,0.07580365],
 [0.02654486,0.12618815,0.07579381],
 [0.02656222,0.12620534,0.10073176],
 [0.02633519,0.12576932,0.10073484],
 [0.02578417,0.12563403,0.10078865],
 [0.0253286 ,0.12600623,0.10081131],
 [0.02532631,0.12657007,0.1007793 ],
 [0.0257289 ,0.12683673,0.10073937],
 [0.02626461,0.12676287,0.10074964],
 [0.02656456,0.12627371,0.1007343 ],
 [0.0266693 ,0.12627728,0.12566169],
 [0.02643636,0.12587195,0.12566004],
 [0.02588281,0.1257329 ,0.12571425],
 [0.02540766,0.12608234,0.12572477],
 [0.02542046,0.12666711,0.12569617],
 [0.02585013,0.12690782,0.1256465 ],
 [0.02635637,0.12682234,0.1256818 ],
 [0.02667056,0.1263776 ,0.12566278],
 [0.02408622,0.22567718,0.07595085],
 [0.02375428,0.22526884,0.0759567 ],
 [0.02319062,0.22534668,0.07600733],
 [0.02282968,0.22576215,0.07604664],
 [0.02296259,0.2263154 ,0.07598031],
 [0.02348206,0.22652512,0.07594   ],
 [0.02393295,0.22628852,0.07592726],
 [0.02408351,0.22571636,0.07595247],
 [0.0242098 ,0.22573392,0.10092291],
 [0.02386377,0.22534117,0.10092377],
 [0.02331787,0.22538125,0.10098214],
 [0.02293914,0.22581575,0.10101873],
 [0.02308012,0.22637342,0.10094587],
 [0.02358661,0.22657412,0.10090667],
 [0.02404174,0.22634975,0.10091137],
 [0.0241947 ,0.2257635 ,0.10092219],
 [0.02440222,0.2257337 ,0.1258737 ],
 [0.02407727,0.22537377,0.12586656],
 [0.02352753,0.22540279,0.12591885],
 [0.02315022,0.2258092 ,0.1259644 ],
 [0.0232708 ,0.22639415,0.12588764],
 [0.02378642,0.22656046,0.12585163],
 [0.02422347,0.22634327,0.12585937],
 [0.0243944 ,0.22578098,0.12587466],
 [0.05872868,0.02754342,0.07542343],
 [0.05861548,0.02710631,0.07546537],
 [0.05820119,0.02681743,0.07552961],
 [0.05760851,0.02698172,0.07548053],
 [0.05740931,0.02754825,0.07547431],
 [0.05775348,0.02793052,0.07543757],
 [0.05829473,0.0279786 ,0.07541465],
 [0.05871297,0.02761111,0.07542296],
 [0.05887565,0.02746938,0.10037374],
 [0.05877306,0.02702588,0.10040061],
 [0.05833938,0.02674695,0.10047739],
 [0.05776053,0.02690557,0.10043711],
 [0.05758202,0.02747334,0.10040526],
 [0.05794306,0.02782455,0.10036994],
 [0.05848458,0.02788126,0.10037846],
 [0.05884308,0.02754155,0.10037784],
 [0.05904608,0.02742705,0.12535083],
 [0.05895937,0.02695225,0.12536898],
 [0.05853382,0.02664086,0.12544711],
 [0.05787802,0.02686007,0.12538953],
 [0.05775636,0.02739348,0.12536107],
 [0.05809727,0.0277984 ,0.12531818],
 [0.05857626,0.0278791 ,0.12535467],
 [0.05901249,0.02747795,0.12534545],
 [0.05639872,0.1268544 ,0.07569694],
 [0.05613523,0.12641054,0.07569698],
 [0.05559877,0.12625557,0.07573327],
 [0.05511613,0.12659808,0.07572729],
 [0.05511741,0.12710628,0.07572827],
 [0.0555129 ,0.12747388,0.07570326],
 [0.05600312,0.12732785,0.07569224],
 [0.05636112,0.12692651,0.07569823],
 [0.05644253,0.12683338,0.10061092],
 [0.05613956,0.12652381,0.10062872],
 [0.05564298,0.12632529,0.10065974],
 [0.05511581,0.12669815,0.10066368],
 [0.05515335,0.12719077,0.10066161],
 [0.05556719,0.12751198,0.10062158],
 [0.05605435,0.12747561,0.10062152],
 [0.05638425,0.12700432,0.10063542],
 [0.0566449 ,0.12688227,0.12553672],
 [0.05634975,0.12657603,0.12555628],
 [0.05582549,0.12641089,0.12559296],
 [0.05534179,0.12673572,0.12559473],
 [0.05529441,0.12730744,0.12558687],
 [0.0557356 ,0.12760752,0.12554011],
 [0.0561982 ,0.12753727,0.12556426],
 [0.05654453,0.12710827,0.12556489],
 [0.0540275 ,0.22632775,0.07588915],
 [0.05367572,0.22599809,0.07589543],
 [0.05310376,0.2259856 ,0.07592964],
 [0.05276682,0.22641717,0.07595893],
 [0.05287826,0.22696922,0.07591634],
 [0.05338681,0.22719796,0.07586514],
 [0.05383428,0.22696626,0.07584563],
 [0.05402742,0.22634551,0.07589159],
 [0.0541255 ,0.2264164 ,0.10083523],
 [0.05378876,0.2261144 ,0.10083631],
 [0.05323429,0.2260905 ,0.10087868],
 [0.05287258,0.22651012,0.10090356],
 [0.05297864,0.22710347,0.10085075],
 [0.05348029,0.22731727,0.1008056 ],
 [0.05394905,0.22706089,0.10079715],
 [0.05412662,0.2264851 ,0.100835  ],
 [0.05425967,0.22645491,0.12579162],
 [0.05392986,0.22613137,0.12579596],
 [0.05335847,0.22608605,0.12585083],
 [0.05299338,0.22652672,0.12586302],
 [0.05309099,0.22710049,0.12581051],
 [0.05359677,0.22731529,0.12577209],
 [0.05404241,0.22709789,0.12576349],
 [0.0542474 ,0.22650082,0.12579294],
 [0.08864083,0.02831224,0.07532355],
 [0.08865151,0.02777023,0.07538382],
 [0.08822915,0.02750793,0.07542632],
 [0.08762968,0.02759555,0.0753639 ],
 [0.0873541 ,0.02815802,0.07538252],
 [0.08767161,0.02852213,0.07533959],
 [0.08820561,0.02865912,0.0753355 ],
 [0.08862686,0.0283449 ,0.07532319],
 [0.08868106,0.02820369,0.10023712],
 [0.08866293,0.02765607,0.10026475],
 [0.08826563,0.02737494,0.10032612],
 [0.08764016,0.02750106,0.10026508],
 [0.08741581,0.02806166,0.10026902],
 [0.08770515,0.02843464,0.10024243],
 [0.08823801,0.0285545 ,0.10023411],
 [0.08872427,0.02819978,0.1002386 ],
 [0.08873954,0.02817392,0.12520839],
 [0.08868132,0.02771091,0.12524555],
 [0.08827424,0.02738199,0.125303  ],
 [0.08766322,0.0275047 ,0.12524174],
 [0.08745452,0.02807293,0.12523979],
 [0.08776271,0.02842223,0.12520182],
 [0.0882634 ,0.02855188,0.12521614],
 [0.08874187,0.02821398,0.12520963],
 [0.08630199,0.12764732,0.0756118 ],
 [0.08607935,0.12715068,0.07562152],
 [0.08557679,0.1269423 ,0.07563925],
 [0.08511204,0.12721475,0.07565048],
 [0.08500847,0.12778447,0.07565614],
 [0.08541103,0.12812064,0.07561866],
 [0.08594901,0.12808919,0.07562416],
 [0.08630283,0.1276522 ,0.07561135],
 [0.08635951,0.12767244,0.1005221 ],
 [0.08614893,0.12721098,0.10051695],
 [0.0856725 ,0.12700893,0.10054343],
 [0.08515783,0.12732268,0.10054888],
 [0.08507973,0.12785867,0.10056579],
 [0.08545776,0.12820007,0.10053738],
 [0.08599937,0.12818963,0.10053576],
 [0.08635964,0.12775969,0.10052842],
 [0.08642938,0.12774572,0.12545522],
 [0.08622606,0.12726461,0.12545259],
 [0.0857142 ,0.12709874,0.12547959],
 [0.08524726,0.1273581 ,0.12547475],
 [0.08517768,0.12791472,0.12549135],
 [0.08551709,0.12825359,0.12545458],
 [0.08606882,0.12824805,0.1254631 ],
 [0.08643356,0.12778923,0.12545565],
 [0.08391826,0.22726365,0.07579955],
 [0.08357971,0.22689029,0.07579798],
 [0.08302909,0.22685453,0.07582337],
 [0.08265545,0.22726992,0.07583431],
 [0.08275424,0.22782726,0.07580538],
 [0.08327566,0.22811275,0.07576001],
 [0.08368387,0.22784061,0.07573845],
 [0.08391918,0.22727095,0.07579829],
 [0.08396097,0.22718342,0.10076524],
 [0.08364424,0.22681594,0.10075376],
 [0.08308216,0.22681905,0.1007802 ],
 [0.08269483,0.22721322,0.10079793],
 [0.0827941 ,0.22778974,0.1007713 ],
 [0.08332158,0.22804596,0.10071974],
 [0.08372579,0.22779164,0.10070936],
 [0.08395557,0.22724871,0.1007669 ],
 [0.08409473,0.22718117,0.12572593],
 [0.08377872,0.2268328 ,0.12570782],
 [0.08321463,0.22678137,0.12573558],
 [0.08282585,0.22719163,0.12575374],
 [0.08292356,0.2277393 ,0.12571959],
 [0.08344711,0.2280378 ,0.12567353],
 [0.08385668,0.22777896,0.12566837],
 [0.08408788,0.22723243,0.12572059]]
 
robotpts = [[ 0.35999518 ,-0.24995843 ,-0.06402115+0.15],
 [ 0.35998147 ,-0.24993273 ,-0.06402168+0.15],
 [ 0.35997729 ,-0.24992881 ,-0.06402209+0.15],
 [ 0.35997978 ,-0.24993224 ,-0.06402209+0.15],
 [ 0.35998033 ,-0.24993299 ,-0.06402209+0.15],
 [ 0.35997729 ,-0.24992881 ,-0.06402209+0.15],
 [ 0.35997555 ,-0.24992642 ,-0.06402209+0.15],
 [ 0.35998077 ,-0.24992925 ,-0.06402209+0.15],
 [ 0.35996794 ,-0.249991   ,-0.0390174 +0.15],
 [ 0.35995183 ,-0.24994948 ,-0.03901582+0.15],
 [ 0.35995004 ,-0.24995039 ,-0.03901646+0.15],
 [ 0.35994435 ,-0.24994209 ,-0.03901646+0.15],
 [ 0.35994789 ,-0.24995438 ,-0.03901594+0.15],
 [ 0.35994881 ,-0.24995164 ,-0.03901594+0.15],
 [ 0.35994716 ,-0.24995127 ,-0.03901582+0.15],
 [ 0.35994151 ,-0.24993896 ,-0.03901582+0.15],
 [ 0.36006128 ,-0.25007949 ,-0.01398776+0.15],
 [ 0.36003714 ,-0.25003541 ,-0.01398846+0.15],
 [ 0.36003566 ,-0.2500393  ,-0.01398834+0.15],
 [ 0.3600382  ,-0.25004316 ,-0.01398834+0.15],
 [ 0.36003621 ,-0.25004014 ,-0.01398834+0.15],
 [ 0.36003498 ,-0.25003733 ,-0.01398834+0.15],
 [ 0.36003918 ,-0.25004372 ,-0.01398834+0.15],
 [ 0.36003664 ,-0.25003985 ,-0.01398834+0.15],
 [ 0.36006801 ,-0.15015701 ,-0.06398368+0.15],
 [ 0.36005965 ,-0.15012747 ,-0.0639846 +0.15],
 [ 0.36005926 ,-0.15013129 ,-0.0639843 +0.15],
 [ 0.36006165 ,-0.15013604 ,-0.06398459+0.15],
 [ 0.36005952 ,-0.15013064 ,-0.0639846 +0.15],
 [ 0.36006092 ,-0.15013418 ,-0.0639846 +0.15],
 [ 0.36005746 ,-0.15012541 ,-0.0639846 +0.15],
 [ 0.36006184 ,-0.15013304 ,-0.0639846 +0.15],
 [ 0.36005719 ,-0.15007465 ,-0.03898689+0.15],
 [ 0.36003995 ,-0.15003666 ,-0.03898689+0.15],
 [ 0.36004238 ,-0.15004302 ,-0.03898689+0.15],
 [ 0.3600371  ,-0.15003264 ,-0.03898702+0.15],
 [ 0.36004175 ,-0.15004484 ,-0.03898702+0.15],
 [ 0.36004305 ,-0.15004479 ,-0.03898689+0.15],
 [ 0.36004042 ,-0.15003406 ,-0.03898779+0.15],
 [ 0.36004378 ,-0.15005015 ,-0.03898702+0.15],
 [ 0.35996596 ,-0.15002227 ,-0.01399506+0.15],
 [ 0.35994986 ,-0.14997225 ,-0.0139946 +0.15],
 [ 0.35994944 ,-0.14996721 ,-0.01399553+0.15],
 [ 0.35995336 ,-0.14997769 ,-0.01399553+0.15],
 [ 0.35995125 ,-0.14996842 ,-0.01399553+0.15],
 [ 0.35995378 ,-0.14997695 ,-0.01399598+0.15],
 [ 0.35995383 ,-0.14997093 ,-0.01399629+0.15],
 [ 0.35995657 ,-0.14998063 ,-0.01399644+0.15],
 [ 0.35999006 ,-0.05013934 ,-0.06402222+0.15],
 [ 0.35998421 ,-0.05010943 ,-0.06402255+0.15],
 [ 0.35998166 ,-0.05010159 ,-0.06402208+0.15],
 [ 0.35997962 ,-0.05010393 ,-0.06402145+0.15],
 [ 0.35997807 ,-0.05009172 ,-0.06402145+0.15],
 [ 0.35998154 ,-0.05010276 ,-0.06402207+0.15],
 [ 0.35998105 ,-0.05009887 ,-0.06402207+0.15],
 [ 0.3599789  ,-0.05010026 ,-0.0640213 +0.15],
 [ 0.35999411 ,-0.05008874 ,-0.03900393+0.15],
 [ 0.35998722 ,-0.05005198 ,-0.03900414+0.15],
 [ 0.3599884  ,-0.0500547  ,-0.03900434+0.15],
 [ 0.3599884  ,-0.0500547  ,-0.03900434+0.15],
 [ 0.35998828 ,-0.05004761 ,-0.03900434+0.15],
 [ 0.35998598 ,-0.05004764 ,-0.03900384+0.15],
 [ 0.35998796 ,-0.0500512  ,-0.03900434+0.15],
 [ 0.35998685 ,-0.05005464 ,-0.03900384+0.15],
 [ 0.3600575  ,-0.05004586 ,-0.01400766+0.15],
 [ 0.36005485 ,-0.05001319 ,-0.01400797+0.15],
 [ 0.3600539  ,-0.05000557 ,-0.01400797+0.15],
 [ 0.36005336 ,-0.05001128 ,-0.01400727+0.15],
 [ 0.36005282 ,-0.05001309 ,-0.01400727+0.15],
 [ 0.36005017 ,-0.05000542 ,-0.01400755+0.15],
 [ 0.36005336 ,-0.05001128 ,-0.01400727+0.15],
 [ 0.36005112 ,-0.05001303 ,-0.01400755+0.15],
 [ 0.38995876 ,-0.24991727 ,-0.06403405+0.15],
 [ 0.38995576 ,-0.24989492 ,-0.06403462+0.15],
 [ 0.38995122 ,-0.24989216 ,-0.06403411+0.15],
 [ 0.3899535  ,-0.24989897 ,-0.0640337 +0.15],
 [ 0.38995146 ,-0.24989571 ,-0.06403362+0.15],
 [ 0.38995292 ,-0.24989501 ,-0.0640342 +0.15],
 [ 0.38995111 ,-0.24989617 ,-0.0640337 +0.15],
 [ 0.3899519  ,-0.24989225 ,-0.06403503+0.15],
 [ 0.3900561  ,-0.25003943 ,-0.0390082 +0.15],
 [ 0.39004222 ,-0.25001313 ,-0.03900806+0.15],
 [ 0.39003692 ,-0.25000535 ,-0.03900875+0.15],
 [ 0.39004201 ,-0.25001166 ,-0.03900875+0.15],
 [ 0.39004051 ,-0.2500098  ,-0.03900875+0.15],
 [ 0.39004317 ,-0.2500131  ,-0.03900875+0.15],
 [ 0.39003794 ,-0.25000784 ,-0.03900875+0.15],
 [ 0.39004387 ,-0.25001396 ,-0.03900875+0.15],
 [ 0.39005591 ,-0.25003835 ,-0.01398419+0.15],
 [ 0.39004117 ,-0.25000987 ,-0.01398379+0.15],
 [ 0.39003251 ,-0.25000137 ,-0.01398403+0.15],
 [ 0.39004001 ,-0.25001004 ,-0.01398379+0.15],
 [ 0.39003713 ,-0.25001008 ,-0.01398384+0.15],
 [ 0.39003442 ,-0.25000658 ,-0.01398384+0.15],
 [ 0.39003807 ,-0.2500096  ,-0.01398428+0.15],
 [ 0.39003807 ,-0.25001021 ,-0.01398403+0.15],
 [ 0.38997583 ,-0.15015172 ,-0.06399709+0.15],
 [ 0.38996394 ,-0.15012706 ,-0.06399682+0.15],
 [ 0.38996602 ,-0.15012841 ,-0.06399716+0.15],
 [ 0.38996602 ,-0.15012841 ,-0.06399716+0.15],
 [ 0.38996723 ,-0.15013101 ,-0.06399716+0.15],
 [ 0.3899622  ,-0.15012016 ,-0.06399716+0.15],
 [ 0.3899652  ,-0.15012895 ,-0.06399746+0.15],
 [ 0.3899652  ,-0.15013028 ,-0.06399716+0.15],
 [ 0.38996863 ,-0.15006601 ,-0.03900468+0.15],
 [ 0.38996427 ,-0.1500425  ,-0.03900395+0.15],
 [ 0.38995531 ,-0.15003256 ,-0.0390036 +0.15],
 [ 0.3899609  ,-0.15004494 ,-0.0390036 +0.15],
 [ 0.38995638 ,-0.15003353 ,-0.03900347+0.15],
 [ 0.38996028 ,-0.15004054 ,-0.03900412+0.15],
 [ 0.38995973 ,-0.15003602 ,-0.0390045 +0.15],
 [ 0.38996059 ,-0.15004286 ,-0.03900347+0.15],
 [ 0.38996475 ,-0.15001262 ,-0.01400049+0.15],
 [ 0.389956   ,-0.14998285 ,-0.01400015+0.15],
 [ 0.3899594  ,-0.14998721 ,-0.01400055+0.15],
 [ 0.38996412 ,-0.14999051 ,-0.01400167+0.15],
 [ 0.38996195 ,-0.14998565 ,-0.01400167+0.15],
 [ 0.38995813 ,-0.14998252 ,-0.014001  +0.15],
 [ 0.38996412 ,-0.14999051 ,-0.01400167+0.15],
 [ 0.38996068 ,-0.1499828  ,-0.01400167+0.15],
 [ 0.39003141 ,-0.0501371  ,-0.06398769+0.15],
 [ 0.39001663 ,-0.05011026 ,-0.06398781+0.15],
 [ 0.39001796 ,-0.05011559 ,-0.06398794+0.15],
 [ 0.3900195  ,-0.05011717 ,-0.06398794+0.15],
 [ 0.39002298 ,-0.05011714 ,-0.0639877 +0.15],
 [ 0.39002186 ,-0.05010974 ,-0.0639877 +0.15],
 [ 0.39002    ,-0.0501151  ,-0.06398809+0.15],
 [ 0.3900199  ,-0.05011645 ,-0.06398794+0.15],
 [ 0.39005429 ,-0.05005839 ,-0.039     +0.15],
 [ 0.39005604 ,-0.05003076 ,-0.03900033+0.15],
 [ 0.39005131 ,-0.05002275 ,-0.03899981+0.15],
 [ 0.3900549  ,-0.05002923 ,-0.03900053+0.15],
 [ 0.39005087 ,-0.05002677 ,-0.03899981+0.15],
 [ 0.39005388 ,-0.05002938 ,-0.03900053+0.15],
 [ 0.39005062 ,-0.05002509 ,-0.03899981+0.15],
 [ 0.39005102 ,-0.05002778 ,-0.03899981+0.15],
 [ 0.39006473 ,-0.05003086 ,-0.01398538+0.15],
 [ 0.39005378 ,-0.05000578 ,-0.01398365+0.15],
 [ 0.39005097 ,-0.04999737 ,-0.01398319+0.15],
 [ 0.39005063 ,-0.05000376 ,-0.01398365+0.15],
 [ 0.39004965 ,-0.04999716 ,-0.01398365+0.15],
 [ 0.39005063 ,-0.05000131 ,-0.01398385+0.15],
 [ 0.39005079 ,-0.05000481 ,-0.01398365+0.15],
 [ 0.390049   ,-0.04999035 ,-0.01398385+0.15],
 [ 0.42009248 ,-0.24994306 ,-0.0639982 +0.15],
 [ 0.42008291 ,-0.24991805 ,-0.06399971+0.15],
 [ 0.42007773 ,-0.24991966 ,-0.06399896+0.15],
 [ 0.42007921 ,-0.24992091 ,-0.06399963+0.15],
 [ 0.42008131 ,-0.24992101 ,-0.06399846+0.15],
 [ 0.42007642 ,-0.2499235  ,-0.06399938+0.15],
 [ 0.42008335 ,-0.24992575 ,-0.06399871+0.15],
 [ 0.42007463 ,-0.24992111 ,-0.06399963+0.15],
 [ 0.42004954 ,-0.25007105 ,-0.03903392+0.15],
 [ 0.42003891 ,-0.25004632 ,-0.03903192+0.15],
 [ 0.42004071 ,-0.25004763 ,-0.03903307+0.15],
 [ 0.42003706 ,-0.25004641 ,-0.03903207+0.15],
 [ 0.42003499 ,-0.25004427 ,-0.03903207+0.15],
 [ 0.4200366  ,-0.2500496  ,-0.03903174+0.15],
 [ 0.42004217 ,-0.2500517  ,-0.03903207+0.15],
 [ 0.42003499 ,-0.25004427 ,-0.03903207+0.15],
 [ 0.41990389 ,-0.25006499 ,-0.01398447+0.15],
 [ 0.41989371 ,-0.25003026 ,-0.01398444+0.15],
 [ 0.41989811 ,-0.25003134 ,-0.01398507+0.15],
 [ 0.419894   ,-0.2500269  ,-0.01398507+0.15],
 [ 0.41990235 ,-0.25003591 ,-0.01398507+0.15],
 [ 0.41990235 ,-0.25003591 ,-0.01398507+0.15],
 [ 0.41989811 ,-0.25003073 ,-0.01398532+0.15],
 [ 0.41989455 ,-0.25003164 ,-0.01398507+0.15],
 [ 0.42006052 ,-0.15014403 ,-0.06397916+0.15],
 [ 0.42004502 ,-0.15012399 ,-0.06397917+0.15],
 [ 0.42004544 ,-0.15012011 ,-0.0639801 +0.15],
 [ 0.42004544 ,-0.15012011 ,-0.0639801 +0.15],
 [ 0.42004769 ,-0.15012417 ,-0.0639801 +0.15],
 [ 0.42004709 ,-0.15012309 ,-0.0639801 +0.15],
 [ 0.42004927 ,-0.15012196 ,-0.06397961+0.15],
 [ 0.42004484 ,-0.15011903 ,-0.0639801 +0.15],
 [ 0.42001502 ,-0.15006804 ,-0.03900352+0.15],
 [ 0.4200163  ,-0.15004966 ,-0.03900453+0.15],
 [ 0.42001366 ,-0.15004946 ,-0.03900401+0.15],
 [ 0.42001366 ,-0.15004562 ,-0.03900491+0.15],
 [ 0.42001316 ,-0.15004665 ,-0.03900344+0.15],
 [ 0.42001226 ,-0.15004714 ,-0.03900344+0.15],
 [ 0.4200133  ,-0.15005066 ,-0.0390038 +0.15],
 [ 0.42001226 ,-0.15004331 ,-0.03900434+0.15],
 [ 0.41992934 ,-0.15000169 ,-0.01400089+0.15],
 [ 0.41992118 ,-0.14997141 ,-0.01400079+0.15],
 [ 0.41992672 ,-0.14997953 ,-0.01400158+0.15],
 [ 0.41992412 ,-0.14997224 ,-0.01400134+0.15],
 [ 0.41992576 ,-0.14997994 ,-0.01400079+0.15],
 [ 0.41992178 ,-0.14997495 ,-0.01400103+0.15],
 [ 0.41992612 ,-0.14997598 ,-0.01400134+0.15],
 [ 0.41992226 ,-0.14997585 ,-0.01400103+0.15],
 [ 0.42001515 ,-0.05000304 ,-0.06399976+0.15],
 [ 0.42000524 ,-0.04998529 ,-0.06399957+0.15],
 [ 0.42000657 ,-0.04998526 ,-0.06399931+0.15],
 [ 0.42000628 ,-0.04998368 ,-0.06399931+0.15],
 [ 0.42000603 ,-0.04998225 ,-0.06399931+0.15],
 [ 0.42000337 ,-0.04998028 ,-0.06399995+0.15],
 [ 0.42000435 ,-0.04998573 ,-0.06399995+0.15],
 [ 0.42000892 ,-0.04998354 ,-0.06399964+0.15],
 [ 0.42003812 ,-0.05007876 ,-0.03898386+0.15],
 [ 0.42002605 ,-0.05004621 ,-0.03898355+0.15],
 [ 0.42002399 ,-0.05004846 ,-0.03898305+0.15],
 [ 0.42002408 ,-0.05003994 ,-0.03898341+0.15],
 [ 0.42002561 ,-0.05004851 ,-0.03898341+0.15],
 [ 0.42002213 ,-0.05004706 ,-0.03898269+0.15],
 [ 0.42002069 ,-0.05003895 ,-0.03898269+0.15],
 [ 0.4200251  ,-0.05004565 ,-0.03898341+0.15],
 [ 0.42006178 ,-0.05003264 ,-0.01398789+0.15],
 [ 0.42005654 ,-0.05000387 ,-0.01398739+0.15],
 [ 0.42005453 ,-0.05000669 ,-0.01398719+0.15],
 [ 0.42005416 ,-0.05001349 ,-0.0139868 +0.15],
 [ 0.42005099 ,-0.050009   ,-0.01398726+0.15],
 [ 0.42005149 ,-0.05000558 ,-0.01398706+0.15],
 [ 0.42005557 ,-0.05001245 ,-0.01398719+0.15],
 [ 0.42005601 ,-0.05000336 ,-0.01398719+0.15]]

viconpts = np.array(viconpts)
robotpts = np.array(robotpts)
print 'viconpts', viconpts
print 'robotpts', robotpts

(R,t,rmse,err) = rigid_transform_3D(viconpts, robotpts)  # then you'll get vicon frame wrt robot frame

while True:  # 
    
    import pdb; pdb.set_trace()
    #print err
    err = np.linalg.norm(err, axis=1)

    thres = 0.1
    maxind = maxindex(err)
    filtered_viconpts = np.array([pts for i,pts in enumerate(viconpts.tolist()) if err[i] < thres and i != maxind])
    filtered_robotpts = np.array([pts for i,pts in enumerate(robotpts.tolist()) if err[i] < thres and i != maxind])

    plt.scatter(filtered_viconpts[:,0], filtered_viconpts[:,1], c='r', marker='o')
    plt.scatter(filtered_robotpts[:,0], filtered_robotpts[:,1], c='b', marker='o')

    plt.show()

    (R,t,rmse,err) = rigid_transform_3D(filtered_viconpts, filtered_robotpts)  # then you'll get vicon frame wrt robot frame

    Rh = tfm.identity_matrix()
    Rh[np.ix_([0,1,2],[0,1,2])] = R
    quat = tfm.quaternion_from_matrix(Rh)

    print 'vicon_T_robot:', " ".join('%.8e' % x for x in (t.tolist() + quat.tolist()))
    print 'rmse:', rmse

    viconpts = filtered_viconpts
    robotpts = filtered_robotpts
