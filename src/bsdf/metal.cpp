#include "metal.hpp"

namespace yart {

static constexpr float E_msLut[32][32] = {
  {0.9635857939720154, 0.995747983455658,  0.9983890056610107, 0.9992253184318542, 0.9995855093002319, 0.9996441006660461, 0.9998587369918823, 0.99983811378479,   0.9998855590820312, 0.9998201727867126, 0.9998736381530762, 0.9998807311058044, 0.9998859763145447, 0.9998285174369812, 0.9999691247940063, 0.9998969435691833,  0.9999278783798218,  0.9999445676803589, 0.999960720539093,   0.9999779462814331,  0.9999331831932068,  0.9999798536300659,  0.999980628490448,   0.9999813437461853,  0.9999666213989258,  0.9999518990516663,  0.999967634677887,   0.9999833106994629,  0.9999531507492065,  0.9999687671661377,  0.9999690651893616,  0.9999998807907104},
  {0.9624530076980591, 0.9955804944038391, 0.9987631440162659, 0.9992542862892151, 0.9995307326316833, 0.9997747540473938, 0.9998279809951782, 0.999792218208313,  0.999916136264801,  0.9996671080589294, 0.9998441338539124, 0.9997889399528503, 0.9998522996902466, 0.9998584389686584, 0.9999077916145325, 0.9999716281890869,  0.9999582767486572,  0.9999406933784485, 0.9999462366104126,  0.999954104423523,   0.9999637007713318,  0.9999645948410034,  0.9999653697013855,  0.999966025352478,   0.9999495148658752,  0.9999824166297913,  0.999937117099762,   0.9999810457229614,  0.9999531507492065,  0.9999687671661377,  0.9999995827674866,  0.999984622001648},
  {0.9328259229660034, 0.990190863609314,  0.9964872598648071, 0.9983128905296326, 0.998899519443512,  0.9993115663528442, 0.9995121955871582, 0.9996293187141418, 0.9997661113739014, 0.9997414350509644, 0.9998139142990112, 0.9997804760932922, 0.9998407363891602, 0.9997414350509644, 0.9997649788856506, 0.9997093081474304,  0.9998055696487427,  0.9998250007629395, 0.9998658895492554,  0.9999068379402161,  0.9998948574066162,  0.9999427199363708,  0.9998825788497925,  0.9999004602432251,  0.9999170303344727,  0.9999791979789734,  0.9999498724937439,  0.9999508857727051,  0.9999822974205017,  0.9999220371246338,  0.9999837875366211,  0.9999691247940063},
  {0.8930054306983948, 0.963340699672699,  0.9859972596168518, 0.9929569363594055, 0.996091365814209,  0.9974014759063721, 0.998002827167511,  0.9985195398330688, 0.9987627267837524, 0.9990635514259338, 0.999199390411377,  0.9992281198501587, 0.9993909001350403, 0.9995043873786926, 0.9994504451751709, 0.9996720552444458,  0.9996840357780457,  0.9995111227035522, 0.9997578859329224,  0.9997982382774353,  0.9997780919075012,  0.9995821714401245,  0.9997304677963257,  0.9997903108596802,  0.9998418092727661,  0.9998234510421753,  0.9997690916061401,  0.9998010993003845,  0.9998071193695068,  0.9998663067817688,  0.9998665452003479,  0.9998453259468079},
  {0.8982181549072266, 0.9266762137413025, 0.9636628031730652, 0.98019939661026,   0.9873616695404053, 0.9918653964996338, 0.9942402243614197, 0.9956909418106079, 0.996512234210968,  0.9973861575126648, 0.9977238774299622, 0.9982470870018005, 0.998320996761322,  0.9985633492469788, 0.9987225532531738, 0.998935341835022,   0.9990522861480713,  0.9990097880363464, 0.9990995526313782,  0.9992167353630066,  0.9993321895599365,  0.9992751479148865,  0.9993900060653687,  0.9994798898696899,  0.9994648098945618,  0.9994661808013916,  0.9993231296539307,  0.9995315074920654,  0.9995622038841248,  0.9995185732841492,  0.9995428919792175,  0.99954754114151},
  {0.9151620268821716, 0.8979942798614502, 0.9348788261413574, 0.9588434100151062, 0.9736459255218506, 0.9823293089866638, 0.9873502850532532, 0.9898392558097839, 0.9922816753387451, 0.993818998336792,  0.9948622584342957, 0.9954609870910645, 0.9965735077857971, 0.9967789649963379, 0.9972547292709351, 0.9973844885826111,  0.9976301193237305,  0.9977455139160156, 0.9979255795478821,  0.9983596801757812,  0.9982226490974426,  0.9983841776847839,  0.9987102150917053,  0.998901903629303,   0.9987770915031433,  0.9991421103477478,  0.9986880421638489,  0.9989207983016968,  0.9987643957138062,  0.9987972974777222,  0.9989362955093384,  0.9991236925125122},
  {0.9330495595932007, 0.8908846378326416, 0.9079879522323608, 0.9335653781890869, 0.9522469639778137, 0.965967059135437,  0.9742348790168762, 0.9801027774810791, 0.9846681356430054, 0.9871715903282166, 0.9900511503219604, 0.9913052320480347, 0.992600679397583,  0.9934586882591248, 0.9945163130760193, 0.9951238632202148,  0.9950239658355713,  0.9956044554710388, 0.996037483215332,   0.9963870644569397,  0.9962829947471619,  0.9968684315681458,  0.9971612095832825,  0.9972022175788879,  0.9974944591522217,  0.9974798560142517,  0.997612714767456,   0.9979535937309265,  0.9980898499488831,  0.9980645179748535,  0.9982076287269592,  0.9981849193572998},
  {0.9453397989273071, 0.8931507468223572, 0.8936023116111755, 0.9110523462295532, 0.930195152759552,  0.9464243054389954, 0.9567159414291382, 0.9660252332687378, 0.972061812877655,  0.9772922396659851, 0.9808592796325684, 0.9842429757118225, 0.985924482345581,  0.9886795282363892, 0.9899022579193115, 0.9898055791854858,  0.9914050698280334,  0.9922351241111755, 0.9925453662872314,  0.9938244819641113,  0.9941720366477966,  0.9945185780525208,  0.9945670366287231,  0.9954984188079834,  0.9955605864524841,  0.9953851699829102,  0.9958810806274414,  0.996248722076416,   0.9964290261268616,  0.9964909553527832,  0.9967811703681946,  0.9969521164894104},
  {0.9544344544410706, 0.9022103548049927, 0.8864280581474304, 0.8952941298484802, 0.9088196158409119, 0.9242581129074097, 0.9367639422416687, 0.947743833065033,  0.9565682411193848, 0.9628447890281677, 0.969174861907959,  0.9732633829116821, 0.9772542119026184, 0.9795423150062561, 0.9819976687431335, 0.9845349788665771,  0.9857139587402344,  0.9868486523628235, 0.9881422519683838,  0.9892463088035583,  0.9899702668190002,  0.9913380146026611,  0.9913980960845947,  0.9914693236351013,  0.9926061034202576,  0.9922316074371338,  0.9932136535644531,  0.9937698245048523,  0.993614673614502,   0.9941533207893372,  0.9941940903663635,  0.9942945837974548},
  {0.9617866277694702, 0.9104193449020386, 0.8869238495826721, 0.8837536573410034, 0.8927567601203918, 0.906217634677887,  0.9172398447990417, 0.9285588264465332, 0.9377991557121277, 0.9461573362350464, 0.953217089176178,  0.9600023031234741, 0.964180052280426,  0.9693463444709778, 0.9721860289573669, 0.9742137789726257,  0.9777005314826965,  0.9793358445167542, 0.9813347458839417,  0.9839100241661072,  0.9844508171081543,  0.984897255897522,   0.9853436350822449,  0.9872106909751892,  0.9869411587715149,  0.9885827898979187,  0.9889422655105591,  0.9895589351654053,  0.990011990070343,   0.9906002283096313,  0.9904730916023254,  0.9904001951217651},
  {0.9664388298988342, 0.9197866916656494, 0.8905799388885498, 0.883156418800354,  0.8827630877494812, 0.8896644711494446, 0.8999561071395874, 0.9085540771484375, 0.918840229511261,  0.9277880787849426, 0.9360264539718628, 0.9443001747131348, 0.9497106671333313, 0.9545363783836365, 0.9582120180130005, 0.9616508483886719,  0.9655871391296387,  0.9689820408821106, 0.9717747569084167,  0.9725849628448486,  0.9756177067756653,  0.97783362865448,    0.9788875579833984,  0.9798064827919006,  0.9813156723976135,  0.9821791052818298,  0.9827338457107544,  0.983195960521698,   0.9844823479652405,  0.9845951199531555,  0.9858988523483276,  0.9854810833930969},
  {0.9694132804870605, 0.9257479310035706, 0.8974544405937195, 0.8827428221702576, 0.8775234818458557, 0.8790539503097534, 0.885047435760498,  0.8924179077148438, 0.8990315794944763, 0.9091772437095642, 0.9173170328140259, 0.9247027635574341, 0.9310639500617981, 0.9368415474891663, 0.9423990249633789, 0.9467107653617859,  0.9521124958992004,  0.9548688530921936, 0.9582838416099548,  0.9622583985328674,  0.9648769497871399,  0.9657583832740784,  0.9693913459777832,  0.9698737263679504,  0.9720104932785034,  0.9729395508766174,  0.9751629829406738,  0.9754497408866882,  0.9765639901161194,  0.9774071574211121,  0.9781448841094971,  0.9796364307403564},
  {0.9719973802566528, 0.9306831359863281, 0.9013428092002869, 0.8835747838020325, 0.8736779689788818, 0.8725164532661438, 0.8706512451171875, 0.8772178888320923, 0.8831602334976196, 0.8910667300224304, 0.8990358114242554, 0.9044378995895386, 0.9113524556159973, 0.9180989861488342, 0.923933207988739,  0.9308220744132996,  0.936453640460968,   0.9393223524093628, 0.9423696398735046,  0.9479364156723022,  0.9496214389801025,  0.9525513052940369,  0.9555701613426208,  0.9576094746589661,  0.9587423801422119,  0.9622700810432434,  0.9637910723686218,  0.9662023782730103,  0.9664798378944397,  0.9684286117553711,  0.969126284122467,   0.9697811007499695},
  {0.9727442264556885, 0.9345206022262573, 0.903929591178894,  0.8853692412376404, 0.8722270131111145, 0.8659079670906067, 0.864499568939209,  0.8654941916465759, 0.8688635230064392, 0.8752998113632202, 0.8800772428512573, 0.8857589960098267, 0.8938597440719604, 0.8992937803268433, 0.9035927653312683, 0.9089774489402771,  0.9148463606834412,  0.9191759824752808, 0.9240384101867676,  0.9290494322776794,  0.9338057637214661,  0.9356728792190552,  0.9390535950660706,  0.9426724314689636,  0.9452378153800964,  0.946725606918335,   0.9497633576393127,  0.9519825577735901,  0.9538668990135193,  0.9557593464851379,  0.9576612114906311,  0.9602978825569153},
  {0.9740354418754578, 0.9360936284065247, 0.9068396091461182, 0.8849396109580994, 0.8705020546913147, 0.8618043065071106, 0.856893002986908,  0.8550428748130798, 0.8556362390518188, 0.856810450553894,  0.8621847033500671, 0.8672357201576233, 0.8719860315322876, 0.877931535243988,  0.8839832544326782, 0.8895988464355469,  0.8940945863723755,  0.8994845151901245, 0.9039913415908813,  0.9069719314575195,  0.911770224571228,   0.9165511727333069,  0.91954106092453,    0.9230077266693115,  0.9257956147193909,  0.9294732213020325,  0.931367039680481,   0.9356260299682617,  0.9380109310150146,  0.9394761919975281,  0.9417126178741455,  0.9439557194709778},
  {0.9739981889724731, 0.9375192523002625, 0.9077275991439819, 0.8862419724464417, 0.8690840601921082, 0.8569494485855103, 0.849721372127533,  0.8470186591148376, 0.8441779613494873, 0.8441599607467651, 0.8484534025192261, 0.8481707572937012, 0.8540400266647339, 0.8571040630340576, 0.8619566559791565, 0.8666855096817017,  0.8700076937675476,  0.8751792907714844, 0.8810882568359375,  0.8854482769966125,  0.8874629139900208,  0.8928744792938232,  0.8970271348953247,  0.9019522070884705,  0.9066221714019775,  0.9097342491149902,  0.9117311835289001,  0.9172227382659912,  0.9188991785049438,  0.9202398061752319,  0.9223057627677917,  0.9261207580566406},
  {0.9727393984794617, 0.9361352920532227, 0.9085666537284851, 0.8859085440635681, 0.8672027587890625, 0.8535550236701965, 0.8440457582473755, 0.8386754989624023, 0.8331448435783386, 0.8296475410461426, 0.8320701718330383, 0.8320614099502563, 0.8323298096656799, 0.8349127769470215, 0.838715672492981,  0.8465678691864014,  0.8484311103820801,  0.8528343439102173, 0.8562976717948914,  0.8612843155860901,  0.8653582334518433,  0.8667944669723511,  0.8734550476074219,  0.8771005272865295,  0.8789620995521545,  0.884890615940094,   0.8878726363182068,  0.8908778429031372,  0.8952533602714539,  0.8985325694084167,  0.9012224674224854,  0.9012179374694824},
  {0.971888542175293,  0.9342792630195618, 0.9053354263305664, 0.8812361359596252, 0.8638301491737366, 0.8498647809028625, 0.838006317615509,  0.8286341428756714, 0.8225927352905273, 0.81831294298172,   0.8161141872406006, 0.8173704743385315, 0.8161450028419495, 0.8175442814826965, 0.8166856169700623, 0.8204942345619202,  0.8225816488265991,  0.8278079628944397, 0.831208348274231,   0.8327621817588806,  0.8355974555015564,  0.844249427318573,   0.8467186689376831,  0.8493362665176392,  0.8558359742164612,  0.857487678527832,   0.8598876595497131,  0.8659135699272156,  0.8665709495544434,  0.8707684278488159,  0.8741219639778137,  0.8765773773193359},
  {0.9710095524787903, 0.9338454604148865, 0.9036157131195068, 0.877607524394989,  0.8578240871429443, 0.8432600498199463, 0.8305211663246155, 0.8208866119384766, 0.8131844401359558, 0.8055910468101501, 0.8015779852867126, 0.7981212735176086, 0.7982315421104431, 0.7956949472427368, 0.7954166531562805, 0.7972602248191833,  0.8004674911499023,  0.8004806637763977, 0.8046951293945312,  0.8071873784065247,  0.8093246817588806,  0.8131558299064636,  0.8173164129257202,  0.8185374140739441,  0.8223480582237244,  0.8259267210960388,  0.8300665020942688,  0.833354651927948,   0.8377440571784973,  0.8400111198425293,  0.8443475365638733,  0.8454246520996094},
  {0.9692757725715637, 0.9289442300796509, 0.8985018134117126, 0.8732277154922485, 0.8531774282455444, 0.8375513553619385, 0.8204666972160339, 0.8106128573417664, 0.7999300956726074, 0.7946382761001587, 0.7874180674552917, 0.7818766236305237, 0.7762027382850647, 0.778640866279602,  0.7773511409759521, 0.7748728394508362,  0.7739964723587036,  0.7777711749076843, 0.7774655818939209,  0.7789250612258911,  0.7829545140266418,  0.7826940417289734,  0.7831856608390808,  0.7891747355461121,  0.7912577390670776,  0.7930949330329895,  0.7984800338745117,  0.7998455762863159,  0.8031460642814636,  0.8041367530822754,  0.8093941807746887,  0.8118780255317688},
  {0.9678109884262085, 0.9257891178131104, 0.8953890800476074, 0.8692840933799744, 0.8453633189201355, 0.8281145691871643, 0.8132898211479187, 0.7980917096138,    0.7865176796913147, 0.778489351272583,  0.7721652388572693, 0.7661490440368652, 0.761102557182312,  0.757210373878479,  0.7516317963600159, 0.7535516619682312,  0.7507131695747375,  0.7495840787887573, 0.7510761022567749,  0.7481979727745056,  0.7509517073631287,  0.753508985042572,   0.7549266815185547,  0.7539642453193665,  0.7587844729423523,  0.759218692779541,   0.7613121271133423,  0.763555109500885,   0.7690894603729248,  0.7680454850196838,  0.7723889946937561,  0.777347207069397},
  {0.9660150408744812, 0.9225035905838013, 0.8882213234901428, 0.8598896265029907, 0.8357637524604797, 0.8175596594810486, 0.8007370829582214, 0.7871596217155457, 0.7766233086585999, 0.7633563876152039, 0.755810022354126,  0.7483093738555908, 0.7413790822029114, 0.7362120151519775, 0.7327252626419067, 0.7272384166717529,  0.7241917848587036,  0.7221750020980835, 0.7233375310897827,  0.720094621181488,   0.7215690016746521,  0.7186169028282166,  0.7187048196792603,  0.7217579483985901,  0.7245560884475708,  0.7235075235366821,  0.7248634696006775,  0.7254680395126343,  0.728445827960968,   0.7316557765007019,  0.7313493490219116,  0.7326483726501465},
  {0.9635928273200989, 0.9175184369087219, 0.8811678886413574, 0.8519929647445679, 0.8290166258811951, 0.8079210519790649, 0.7884736061096191, 0.7743305563926697, 0.7603301405906677, 0.7466346621513367, 0.7341760993003845, 0.7301218509674072, 0.7192374467849731, 0.7129997611045837, 0.7071272730827332, 0.703703761100769,   0.6980226039886475,  0.6942259073257446, 0.6924406290054321,  0.6905739903450012,  0.6901945471763611,  0.6889991760253906,  0.6875786781311035,  0.688380777835846,   0.6873308420181274,  0.6864210963249207,  0.6877439618110657,  0.6873460412025452,  0.6892338395118713,  0.6908681392669678,  0.6908101439476013,  0.6928497552871704},
  {0.9606138467788696, 0.9123191833496094, 0.8734177947044373, 0.8437621593475342, 0.8188437223434448, 0.7935510277748108, 0.776088297367096,  0.763528048992157,  0.7447196245193481, 0.7319169640541077, 0.7186697125434875, 0.7098387479782104, 0.7018877267837524, 0.6930599808692932, 0.6833780407905579, 0.6803670525550842,  0.6719420552253723,  0.6678754687309265, 0.6664694547653198,  0.6577346324920654,  0.6592447757720947,  0.6535136103630066,  0.6533161997795105,  0.65375155210495,    0.6503185033798218,  0.649732232093811,   0.6496500372886658,  0.6476905941963196,  0.6492652893066406,  0.6487405896186829,  0.6486935019493103,  0.6494285464286804},
  {0.9589792490005493, 0.907049298286438,  0.8663603067398071, 0.8338260054588318, 0.8069382905960083, 0.7830407619476318, 0.7615501880645752, 0.7456327676773071, 0.7279130816459656, 0.7121050357818604, 0.7002737522125244, 0.6900581121444702, 0.677975058555603,  0.6671620011329651, 0.6626728177070618, 0.6520846486091614,  0.6455555558204651,  0.6443655490875244, 0.6352166533470154,  0.6321080923080444,  0.6260237097740173,  0.6220016479492188,  0.619264543056488,   0.6140581965446472,  0.6142064332962036,  0.613476574420929,   0.607298731803894,   0.6056899428367615,  0.6075679659843445,  0.6077565550804138,  0.6065330505371094,  0.6052879095077515},
  {0.9557879567146301, 0.9005533456802368, 0.8588030338287354, 0.825423002243042,  0.7951205372810364, 0.770338237285614,  0.7503594756126404, 0.7273393273353577, 0.7100672125816345, 0.6953598856925964, 0.6791097521781921, 0.6675797700881958, 0.6593767404556274, 0.6467294096946716, 0.6401093602180481, 0.6280235648155212,  0.6182970404624939,  0.6143262982368469, 0.6079819798469543,  0.6016060709953308,  0.5938466787338257,  0.5901145339012146,  0.5842138528823853,  0.579093337059021,   0.5782942175865173,  0.5723434090614319,  0.5709050297737122,  0.5662955641746521,  0.5656022429466248,  0.5652378797531128,  0.5615172386169434,  0.5623427033424377},
  {0.952154815196991,  0.8950967192649841, 0.8491784930229187, 0.8132340908050537, 0.7853431701660156, 0.7554711103439331, 0.7335531115531921, 0.7133386135101318, 0.6914552450180054, 0.6771699786186218, 0.6601822376251221, 0.648333728313446,  0.6317825317382812, 0.623458743095398,  0.6104113459587097, 0.6014859080314636,  0.5958805084228516,  0.587882399559021,  0.5781434774398804,  0.5675230622291565,  0.562690258026123,   0.5575666427612305,  0.5529671311378479,  0.5477975010871887,  0.5410810708999634,  0.5358807444572449,  0.5298721790313721,  0.5271962881088257,  0.5252378582954407,  0.523171603679657,   0.5204212665557861,  0.5160762071609497},
  {0.9495982527732849, 0.8873708248138428, 0.8410618305206299, 0.8019973635673523, 0.7691025733947754, 0.7420552372932434, 0.716372013092041,  0.6941580772399902, 0.6743897795677185, 0.6584060788154602, 0.6379660367965698, 0.6232298612594604, 0.6088681817054749, 0.5971347689628601, 0.5859493017196655, 0.5768652558326721,  0.5613412857055664,  0.5551052689552307, 0.5488882064819336,  0.5394439697265625,  0.5308660268783569,  0.5253705382347107,  0.5201205015182495,  0.511916995048523,   0.5061615109443665,  0.5004174113273621,  0.4944411516189575,  0.49030831456184387, 0.48184946179389954, 0.4848729968070984,  0.4802219569683075,  0.4770890772342682},
  {0.9472714066505432, 0.8809747695922852, 0.8306245803833008, 0.7899670004844666, 0.7571653723716736, 0.7262875437736511, 0.700050413608551,  0.6779297590255737, 0.654941201210022,  0.6340962648391724, 0.6164668202400208, 0.6038222312927246, 0.5884117484092712, 0.5741749405860901, 0.5597718954086304, 0.5484079718589783,  0.5381782650947571,  0.5275606513023376, 0.5174355506896973,  0.5067463517189026,  0.49964338541030884, 0.4894179701805115,  0.485622376203537,   0.4781540632247925,  0.47223958373069763, 0.463691383600235,   0.45937180519104004, 0.45384371280670166, 0.4475385546684265,  0.44385814666748047, 0.4391520023345947,  0.43212005496025085},
  {0.9439537525177002, 0.8743938207626343, 0.8199770450592041, 0.7754250168800354, 0.7422483563423157, 0.7083220481872559, 0.6840007901191711, 0.6559059619903564, 0.6352874636650085, 0.6168107390403748, 0.5955669283866882, 0.5801939368247986, 0.5613912343978882, 0.5469002723693848, 0.5342342853546143, 0.5242854356765747,  0.506807804107666,   0.4992485046386719, 0.4875054955482483,  0.4779147207736969,  0.4678249955177307,  0.4620799422264099,  0.4548194706439972,  0.44655734300613403, 0.4404577314853668,  0.4287540316581726,  0.4257289171218872,  0.4141024947166443,  0.4096221327781677,  0.404634565114975,   0.40095895528793335, 0.39724573493003845},
  {0.9396477341651917, 0.865368664264679,  0.810107409954071,  0.7658436894416809, 0.7266921997070312, 0.6943004131317139, 0.6668904423713684, 0.6397178173065186, 0.6153740286827087, 0.5925664901733398, 0.5777354836463928, 0.5549141764640808, 0.540705144405365,  0.5247938632965088, 0.5114121437072754, 0.4967556595802307,  0.48490816354751587, 0.4715813398361206, 0.4616187512874603,  0.4509361982345581,  0.43918880820274353, 0.432326078414917,   0.42018333077430725, 0.4142012596130371,  0.40603402256965637, 0.39709874987602234, 0.3892274498939514,  0.382876455783844,   0.3741002082824707,  0.37226662039756775, 0.36697328090667725, 0.36021992564201355},
  {0.9371616840362549, 0.8578751087188721, 0.7993580102920532, 0.7542229294776917, 0.7148508429527283, 0.6786023378372192, 0.6488820910453796, 0.6179985404014587, 0.5960205793380737, 0.572606086730957,  0.5521941781044006, 0.5315218567848206, 0.5164572596549988, 0.4980998933315277, 0.4836742877960205, 0.46899306774139404, 0.45790210366249084, 0.4433338940143585, 0.43130964040756226, 0.42270514369010925, 0.41120871901512146, 0.39908701181411743, 0.3913979232311249,  0.38395124673843384, 0.3760665953159332,  0.36838075518608093, 0.3593154847621918,  0.35242190957069397, 0.344034880399704,   0.3394070565700531,  0.33127161860466003, 0.3257504403591156}
};

static constexpr float E_msAvgLut[32] = {
  0.9998808541567996, 0.9998751356033608, 0.999749131326098, 0.9992710567894392,
  0.9982443900662474, 0.996493035228923, 0.9937197098624893, 0.9898084800806828,
  0.9844009779044427, 0.9773833737708628, 0.9684953742544167,
  0.9576433806214482, 0.9448006303282455, 0.9298767051077448,
  0.9122511356836185, 0.8930473587242886, 0.8712663453188725,
  0.8477540977182798, 0.8217689249431714, 0.794493677385617, 0.7658798263873905,
  0.7352253056596965, 0.7041580225923099, 0.6725652447785251,
  0.6399820749647915, 0.6076094987802207, 0.5751849029911682,
  0.5431709525873885, 0.5113315780181438, 0.480556503898697,
  0.45143645713687874, 0.42275982978753746
};

[[nodiscard]] static constexpr float E_ms(float cosTheta, float r) noexcept {
  size_t ri = min(size_t(r * 31), 30), cosi = min(size_t(cosTheta * 31), 30);
  float ro = r * 31.0f - float(ri), co = cosTheta * 31.0f - float(cosi);

  const float
    d00 = E_msLut[ri][cosi], d01 = E_msLut[ri][cosi + 1],
    d10 = E_msLut[ri + 1][cosi], d11 = E_msLut[ri + 1][cosi + 1];

  return bilerp(d00, d01, d10, d11, ro, co);
}

[[nodiscard]] static constexpr float E_msAvg(float r) noexcept {
  size_t ri = min(size_t(r * 31), 30);
  float ro = r * 31.0f - float(ri);

  return lerp(E_msAvgLut[ri], E_msAvgLut[ri + 1], ro);
}

MetalBSDF::MetalBSDF(
  const float3& reflectance,
  float roughness,
  float anisotropic,
  float ior
) noexcept
  : m_baseColor(reflectance), m_ior(ior), m_roughness(roughness),
    m_microfacets(roughness, anisotropic) {}

float3 MetalBSDF::fImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return {};

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  if (cosTheta_i == 0 || cosTheta_o == 0) return {};

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return {};
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  const float3 Fss = fresnelSchlick(m_baseColor, absDot(wo, wm));
  const float3 fss = m_microfacets.mdf(wm) * Fss * m_microfacets.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  const float Ess = E_ms(cosTheta_o, m_roughness);
  const float3 fms = fss * m_baseColor * (1.0f - Ess) / Ess;

  return fss + fms;
}

float MetalBSDF::pdfImpl(const float3& wo, const float3& wi) const {
  if (m_microfacets.smooth()) return 0;

  float3 wm = wo + wi;
  if (length2(wm) == 0.0f) return 0;
  wm = normalized(wm.z() < 0.0f ? -wm : wm);

  return m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm));
}

BSDFSample MetalBSDF::sampleImpl(
  const float3& wo,
  const float2& u,
  float uc,
  float uc2
) const {
  if (m_microfacets.smooth()) {
    const float3 F = fresnelSchlick(m_baseColor, wo.z());

    return {
      BSDFSample::Reflected | BSDFSample::Specular,
      F / std::abs(wo.z()),
      float3(),
      float3(-wo.x(), -wo.y(), wo.z()),
      sum(F) / 3.0f
    };
  }

  float3 wm = m_microfacets.sampleVisibleMicrofacet(wo, u);
  float3 wi = reflect(wo, wm);
  if (wo.z() * wi.z() < 0.0f) return {BSDFSample::Absorbed};

  const float pdf = m_microfacets.vmdf(wo, wm) / (4 * absDot(wo, wm));

  const float cosTheta_o = std::abs(wo.z()), cosTheta_i = std::abs(wi.z());
  const float3 Fss = fresnelSchlick(m_baseColor, absDot(wo, wm));
  const float3 fss = m_microfacets.mdf(wm) * Fss * m_microfacets.g(wo, wi) /
                     (4 * cosTheta_o * cosTheta_i);

  const float Ess = E_ms(cosTheta_o, m_roughness);
  const float3 fms = fss * m_baseColor * (1.0f - Ess) / Ess;

  return {
    BSDFSample::Reflected | BSDFSample::Glossy,
    fss + fms,
    float3(),
    wi,
    pdf
  };
}

}
