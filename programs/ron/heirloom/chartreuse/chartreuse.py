#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
CHARTREUSE v0.5.2 - build hook.  Emits chartreuse_pkg.vhd (pure Python, no dependencies).

The data were fitted to the source photo (WIN_20231211_00_38_58) in .heirloom_work/rb_chartreuse
(curve.py / target.py / clean_skel.py / fitgen2.py) and are carried here as data:

  KNOTS   28 x 122 packed 16-bit words, quadratic B-spline control points over the folded half plane
          (x 16 virtual px per knot, |Y| = |y - 460| 24 virtual lines per knot), row-major:
            bits 15..8  F, the shading field: F = q / 204.8 - 0.25 (the colourizer input)
            bits  7..1  S, the signed distance to the photo's white lines, Q.1 px (two's complement)
            bit      0  P, line presence (gates the lines the signed field must close elsewhere)
          v0.5.1: S and P come from the photo's THIN-line skeleton only (ridge width <= 26 px on the
          B channel; the broad cream gloss ridges no longer count as lines and are fitted into F
          instead); F's smoothing is 100x stronger in the fin zone (right, |Y| > ~80), S's and P's 4x,
          so the fins come out as continuous bands; F carries a -0.015 offset.
          Emitted as three ROMs: C_FSA (rows 0..16.x, 2048), C_FSB (1024), C_FSC (512, zero padded).
  CURVE   the photo's colour ramp, RGB along the field (dark green -> lime -> white), arc-length
          parameter t 0..1.  C_PY/C_PU/C_PV: 512 x 8 palette, address = palette(1) & t(8);
          palette 0 = Chartreuse (the photo), 1 = Ember (G->R swapped, warm).  Hardware convention
          (U = Cr, V = Cb), 10-bit / 4.  Luma goes through TONE (photo Y -> drawn Y under the 13/16
          output ceiling, white ~777: darks/mids at the photo's levels, only the top compressed) and is
          pre-boosted 16/13; the top of the Chartreuse ramp keeps at least a pale-lime chroma
          (TOP_CB / TOP_CR) so the capped whites do not read grey.
"""
import os
HERE = os.path.dirname(os.path.abspath(__file__))

KNOTS = (
    "8760866081607a607c60826087608e6095609c60a360aa60b060b660bb60c160c760ca60ce60d160d660d860db60de60"
    "e060e360e560e560e660e760ea60eb60ec60e960dd60d160c160af609f608d627e68702b62eb54c645a03b9e34a03da0"
    "38a012a011a00ba005a20ba241a048a045a039a034a03da04aa05ca870ed77196f3865f359aa529e4ca048a245a042a0"
    "3fa03ca039a037a034a032a031a031a036a03f9e4a9e58b46d0382178c409962a962bb60cb60d860de60e060ce60e260"
    "fe60fd60fc60fc60fb60f860f160e660d960c960b860a560905e7f706e4d60154ed841a0359c37a042a0509865c37e1d"
    "8c38945c8260866082607a607d60826088608d6095609d60a360aa60b160b660bc60c160c560ca60ce60d260d660d860"
    "da60de60df60e260e460e560e860e860ea60ea60ec60e960dd60d060c160b2609f628d5a80726f2663eb55bc45963ca4"
    "2fa03ea03fa009a014a00da0049e059e50a044a046a038a033a03fa24a9e5b9c761d7e386b0b65dd579851a24da04a9e"
    "46a042a03fa03da03aa037a035a032a031a030a035a041a04aa0599a6cf9852f89649b62ac5ebc60c960d760df60e460"
    "c760ea60ff60fa60fc60fc60fc60f860f060e560d960c960b860a4628f5a7f726e0163cf4c9a409c33a2389e43a44f96"
    "65ce83198d389d7880608660826079607c60816088608d6095609c60a360aa60af60b560ba60c060c560c960cd60d160"
    "d460d760d960dd60df60e060e360e560e660e860e860ea60eb60e860dc60ce60bf60af609d628b5a7e706e1a63df51b4"
    "449a3ba2309e41a033a00da017a011a001a223a24ea048a045a037a034a040a24c985ec47bfd7e426b2767e35cae5598"
    "4ea44ba248a0459e42a03fa03ca03aa037a035a033a030a0379e42a24d985cb86ff787298c669f5eaf60bf60ce60dc60"
    "de60d760da60fb60fc60fc60fc60fc60fb60f760ed60e360d560c560b362a05e8a667a526d1759e549a23ea0329e3ba2"
    "459c57a467e5881b9054ac7e81608660806079607b60816087608c6093609b60a160a860ae60b460b960bf60c360c760"
    "cb60cf60d260d660d860db60de60df60e160e360e560e660e760e860e960e660d960cc60bc60ac60996088607b666c0f"
    "61e14e9c439e38a02fa043a233a00ea01aa207a01a9a4d984fa04ba042a034a036a042a2509864dc83078150702f6cfd"
    "61d55ba856944f9c4ba247a245a243a040a03da03aa03aa035a032a03ca2469e549c60d27e098846946aa75cb862c860"
    "d360d960db60e660f960fe60fb60fc60fc60fc60fa60f460e960de60cf60be60ab5e95628366793263fd52c4469636a2"
    "35a03fa24b9659c46ff98a30976abe7e826085607f6079627a60806286608b60916099609f60a760ac60b260b760bc60"
    "c160c660c960cd60d060d360d660d960db60de60df60e160e360e360e560e660e760e460d760c860b860a762955c8468"
    "7748690959d84a9c41a233a038a0399c34a01fa21796259e46aa58be539a49a03ea0339e3ba4469857b269ed8a258566"
    "7656751b6d015fdd58c656a8549850984b9a479e45a244a042a03ba034a038a2429e4d985ac46ef3842990609f62b060"
    "be60cc60d960e660f260fa60fd60fc60fc60fc60fd60fb60f660ed60e260d460c460b15e9d648b667c3a6a1156db48a4"
    "3a9c34a4399e459653ba6ae5851f9854a86ab87c836284607e60765c785e7d5a836089608f6096609d60a260a960ae60"
    "b460b960be60c160c660c960cd60d060d360d560d760d960da60dd60de60df60e160e160e260df60d060c160b1609f5e"
    "8d627e62722863f552b6469639a0319a3aa22fac2ca42e922bbc4bd75be55ef954b2459c37a036a2419e509e5ed47c0b"
    "8f468f688064795c793a761d6eff5eef56d754bc54ac53a2509c4aa240a036a236a43e9c4a9a56c266ed7b1f8c549b66"
    "ad5ebe60d060e160ef60f860fc60fd60fc60fc60fd60fd60fc60f660ee60e360d560c55eb2609f688d607c3a691155df"
    "46b03b9a35a4399c459a55be6de789179e48af66b35eac60846483607d5e767075647d72825e86608c60916099609d60"
    "a460aa60ae60b460b860bd60c060c560c760ca60cd60d060d260d460d660d760d960da60db60dc60dd60d960c860b860"
    "a760945e836a783c690955ce47923da038aa35ae2a9c2b84318055da6d0581326e0f57e54c9c3c9e33a23da04a9659c8"
    "6ffb8e3099689c608f5e8664806a7b5e7a42792b781b70135fff5ce3519842a0379c35a03c96489e57c668f17b238f50"
    "a268b860cd60e060ee60f760fc60fd60fd60fd60fd60fd60fd60fb60f560eb60df60d15ec05eae669a688852762f6205"
    "50d942ac399a379e3e984aa25dc674eb8b179b46a862b064a9669b42846481587c6e693073ed7b407962865e87628c60"
    "926098609d60a260a860ac60b160b460b860bc60c060c360c560c760c960cc60cd60cf60d160d160d360d360d460cf60"
    "bc60ac60995c876a7a4c6e1558e14ea849ae3fa630842f80419c60f67d489a7c873e743159eb4cb23e9632a63ba04896"
    "58c46df78e289a60af64ab5ea8629b5e985e93628d68897681547a335afb4dc23f9435aa35ac3d9c4fb468db7f059130"
    "a65abe68d560e860f460fb60fe60ff60fe60fe60fd60fe60fe60fc60f760ef60e460d65ec660b564a1688e5a7c386913"
    "57eb47c03ca0389c3c9e449c50bc60e1760d883c8c6e887e867c8672864c8b148462825a6f6e7f32a01faa3a81727a5c"
    "855e87608b628f6095609a609f60a360a860ac60af60b260b660b860bb60be60c060c160c360c560c660c760c760c860"
    "c860c260af609b5c886a7b586e155fe352aa418430803c845ee0784085728b7e8f7681526d235cef4cb23d9431a63c9c"
    "48945ac66ef78b289a60b466cd5cbe5ec960b960b25ca3609366833c711159e34ab6399e31a6359042805c9886f1ab0f"
    "c048d064e266f25efc60ff60ff60ff60ff60ff60fe60fe60fe60fc60f860ef60e45ed65ec55eb464a1668e5a7b3e6919"
    "58f148ca3dac37a039a2419e4c9853c859f9642771427348661857d051ae559e67ac84f682587e6c6e4a78ff82627e5e"
    "9c3e7d6c7c66815e875c89608c62906095609a609e60a060a460a860aa60ad60af60b160b360b560b660b760b860b960"
    "b960b960ba60b3609b5e89667f5a6e1765e552c0498860c49432ad7ea07e9c6e9a5c9262814a6f175ceb4aae399631a6"
    "3e984a9c5dd274ff8c34a368bc64d25ccf62bd66cd62ba60a66c8e6678356c0f57e03faa2a8a2d86408a5dba810aa658"
    "c94ade5ee96cf060f75efa60fb60fa60f960f860f860f860f860f760f560ed60e060d15ec064ae6a9c6a8a60773d6513"
    "55e548bc3da03790388c40804b82539256cc53e751e951d94fc547883f803e80479e57c76fe588188025797e6e2275f3"
    "97208f6c90238c3684527c6c7d688262865c895e8c608e629160956098609a609d5e9f5ea160a360a460a660a75ea85e"
    "a960a960a962a860a760a0608b5e7d6a652541f17a1ba9fda462d27eb978ad58ae4ca960906a7b3e670b57dd45a0359a"
    "32a640944da866e3800d9644aa68bc5ec960ce66ce5ed352bb569a6880406b1150db36a026802c8048c46d069039ac7e"
    "c07ece6ad662db62de5ce05ee060e060e060df5ede5edd5edd5edd5edc60db60d560c760b460a368965e8b3c7e196de5"
    "5cc150a647a841b640b746a450d05f00673a6221541349fd44cf41943f8e419e47ca54f968217e398a568d587da07434"
    "6a2167ed56905ba268de6cfd780d812b7f4c7d5e7d6c7f68815c845c875c89628a608c608e6890629162926293609360"
    "9468946494629360925c94609460876071606f608c74b860d258ce66bd64c84ec452bb629d728358722361f950c63d94"
    "309c349c489059c67401932d995ea174ac68b46cb85cba50c466bc7e8e7e6f31600357cd43d261f68c44947e9e6ca162"
    "a958ae60b15eb262b266b366b366b460b45cb35cb25eb164b16ab16cb16ab066b05eb060ab609e609062895c8a078af3"
    "82f576ec6df265f35c01571b58345f4c634d63355c0750dd46b44190419e45c84bef541b5e236c277f20901a9158886c"
    "77806eb067015ec45a9c5c9c5c885f8c62ba66d56f0b78177c2b7c3c7c747b687c6e7c5a7d627f627f4c7f5a8058805e"
    "815c816a8146805e7f588168826a795e79609260b660e25eec5ae160d662d662d35ec362a66c8966793a6b0d55e141ae"
    "309430a242a459a267f56b1b8537883e853f884188408a368e709c7e8d5a711c57ba4c9c5fca69d1641178237e177507"
    "77157a1f7c217c2b7c2b7d2b7d397d3d7e5c7e787e7e7d7e7e7c83668c449427961d965e9a62a160a460a260a0609f66"
    "a150a0509a66947e8e7e806e6d4a5f37571f51f74bcb44ac40984094449647c948194d0d55215d2565186c17752f7b3f"
    "7a54773c71806aed650155ac509c52a456a859a65b965d985f9060be63d167e56de56efb720d73197511761f761f7a7e"
    "786179687657751d76297329780f6af7705a9b60de60fe60f860ea60e262e560de60cb5cb162966c824e761f63f74bc4"
    "3a9a369842a2519c579c5a9459ba5bd658d351c351c357c050e952014dcd40a424802180318e3da43c963d9440923e8e"
    "3fa245b246b647b848ba49ba4ac34acb4ac34ac34c96508e55865d9268a07bd29207a409ad23b15eb862bd60c060be60"
    "ba5eb362ac72a4729c58913883196ff75bdd4bc242a83d983a8c398e3b943e96439647804da453ac54b253d553e553d7"
    "51cf4ea0569a64e46eb266055edc4d924ba44b9e4c9e4e9e50a253a455a25698589659985a905ab05b9e5dd45dd05eed"
    "5ec2619260b05ebe5ac162eb53b257cb8309a53ee970ff5cff60f360f260f060e660d25eba5ea1688b647a346a0752dd"
    "3daa319831a237a438a435aa33ac32ac2c90238029803ab046d64be557c048b6348c2f94429c4cb43aa433943298329a"
    "319e31a02f962e8c2d883292339c32a4319a318a3386388a419050b964e6790f8c449f7caf60ba44c06ac364c65ec660"
    "c260b760a8609b64952d92038ef784e174c766c95dcd58d756df55e954d551c04ed34eeb4e0953105b2e5d2752fb47f7"
    "43d641a840b241d84bd75bc06cd26a1159ca489a47a247a047a249a04aa04b9e4ca04ca24ea44ea44fa44f9c4f9c4f94"
    "4f904f904e924c9c4b964c9c4f9c3e8053d4be4be750ff70fe5cf360f560fa60ec60d962c55eb062986a8252701d56ef"
    "43ba388e369439983a9a39a0379835803380338043a46a04885687396b0b53f540a431923c9c4eb862d96d0962fd61e3"
    "5edf5adb56cf53c755d155d455ed50aa489648804c8c4eca52f259ff662c7531895a996aa266a864ac62ac60aa60a864"
    "a85ea960a7609f6096609366972d9b1f9c30983693428f488c468a4a8640833f7c456f2964395c48563b521d50e74bc0"
    "46aa44a843ca41f140f3450350f75fc06abc67a4528e44a242a041a042a042a043a044a044a044a0449e449e45a045a0"
    "45a244a444a644a643a441a245b23f883b848132e57ef26aee64ee5cf060f160ed60e460cf60bc5ea4648968763265fb"
    "4ec846a6479e4fb452b84dba4fd54f9054aa5eea640e8021977e986a7c1f58f141be3188369048b25cdb7a0b99359a5e"
    "945e8b4e8a4a8a3d974f9a5986476b1561f757e75a0b6f267c50795a73477241743b78357c2d7f2583178b15920d9601"
    "960d9962a460b260bc60c160c45ec662c76ac76ec66ec26abe68b86ab26ca9709d6a9054803e6b29590d4edf48b44696"
    "45ae47d04aed4a2344ff3fef41d94bcf5ad76cd66da6619e4aa641a03ca03ca03ca03ca03ca03da03ca03da03da03da0"
    "3ca03ca03ba03b9e3aa0399c36a03baa34804fc4b77ef27ee34ee35ae860e460e060db60d760ce60b5609b5e806a7019"
    "6eff5fc95edb6bf374016e05680f8d19a5219f6a9a7e9c7e997e8e667f2c5df547d537a8338e44a251c66af39521ac4c"
    "c46aa25ebb62b162ad489330773056d6498257b25fa252fb52d159ed5cdd5ad156d152d14fcb4fc351bc5bbe6dc385f1"
    "9919a730b44cc368d460e260ec60f360f760f65ef05ee85cde5ad35ec662b660a552913c7c1f680356e14abc4498449a"
    "48a84aca49014807480147da42a643b64dc65dd66df07a0674a05da0479e3ea039a038a037a037a037a037a036a036a0"
    "35a035a034a033a032a032a2309e31a434aa328060f8ca7ee360d652dd66dc64d560d160cd60c760c260b66096607f64"
    "704c7b017e2983318f46884c85528766a970be42b364b864b24ca4588b507c1f5bf743cc39a038924bac55d87e07a940"
    "b062b766b35e9968af76874252e0349e2f80378a3fa63ba2379a39a03a8c3a98399a37a0359c35923b9847b65adb74ff"
    "922dae50c458d56ce56cf25efb60ff60ff60ff60ff60f860ea5ed862c56ab1689c4e8733710d5ce54ac442a844904b98"
    "4fbc4ce94af549ef46c844b2449848a24fd75cfd6e217e2f843c8232759e56a043a03aa034a033a032a031a031a031a0"
    "30a030a030a031a031a032a033a034a038a035a82b806804c07ccf3cd060d664cf60c95ec560c060ba60b360ac609960"
    "7f607d5e826a865e8d6e956682688d66ac6abb4adf23b84cb366c85ca570855a771d56f13fbc379638964da85adb8809"
    "a63e9b389d37aa7eaf58a42e59c033803084369e3aaa37a43498349637983e9640a63ca437a034a2379845b25cd77bff"
    "9c1fb84ad052e439f01bf62ffc5eff60ff60fe60fa60f260e55ed360be68a9649748861576ed66c758b04db047c04eda"
    "5cfc64345f3f50ff45e743c6459649b051dc5e076c1d78f484188b2a86307b2a6da050a041a036a030a030a031a032a0"
    "33a034a035a036a037a038a03aa03ca03da0409e3baa2a806c06ca7ecc4ac858c664bf5eb860b460ac60a5609d609660"
    "8c608a608f6098609e5ea960ae5c975ec35efc5cf45ef04ee866da4ebd5a97667a586b0d58d53b9e349a3e9c53a66eed"
    "8d279f5090629276855a65da66c655c344b058ae58b2579a56aa5aaa5ec062ba64e558cc48983c9e3aa6419657a87dff"
    "a621c95ae15aed66f45af85af95af75af560f260ee60e960e060cf60b860a168904e8b1789f385d081067d3e785a726c"
    "6c726a50662b5df550b8469c46984bac4fef542160456c3f753179277a2976316f1866f864a04ea040a034a031a033a0"
    "35a036a036a037a039a039a03ba03ca03da03fa041a042a2319e3f8ea47ec25cc060c166bc60b560b060a860a1609a60"
    "926089608a609b609f60a660ae60b560a060c962ff60f560f462eb66d662bd64a362886a74285eed45b435983ba24a96"
    "65d28617994a906e8f7e6ce848804a8061d06d1f71216ef56b056ef9721374177835732560fd4cc63c9839a6419a5782"
    "870ebe5ee15ef16af662f660f360ef64eb64e662e160dd60d860d160c460ad6092648156820994f5a325ad68ad7aa670"
    "9b6a8c6e7c486a0756d146a24096449a4e9a53be54fb55e35ad75fcd62c961e75ceb52a44e984dba5ea04ca03ea033a0"
    "33a036a035a036a038a03aa03ba03ca03da03ea041a042a0439e42a82e845fe4b67ebb50bf64b95eb360ae60a760a060"
    "98609060886084609260a460a760af60b960af60b660f760fb60f960f360e25ecf5eb75e99627d50690f53e43d9a35a0"
    "429e55b671f58f20975c957e64c83f8044a45fcc7601862e7b5e77387b5a7c647d608162804e6e1f53e13e98379c3da6"
    "4b827a0abb7ee46af160f25eef60eb60e660e160dc60d760d260cd60c760bf60af6096607f6279628e3eb144c760ce68"
    "c960b75ca0628854721159d744a43a9a3ea048a24f9e509a4ea04b964c964e964e944da248a43f96399839a05ba04ca0"
    "3ea033a034a036a036a038a039a03ba03ba03da03ea040a042a043a2469c40ac2e807018b97eba56bc64b65eb360ab60"
    "a4609d6095608d60876084609660a860ac60b360bc60ac60cb60fc60fa60f960f160e160cc60b26093647b6466ed50b0"
    "3a9a37a444a05a8873008e729e6a8456468044b8499a60a276ff8a5e7f687e667f64825e846088647d5a65ff4dc23a9a"
    "38a23fa0548c9570d568ed58ef60eb60e760e460e060db60d660d160cc60c760c160b860a7608e607b5e7d629e68c868"
    "da62db5ece5eb66099667e3a66f94dc43b9a38a041a24aa04ca04aa0469e44a044a046a247a0499e449e3ba233a22d9c"
    "5ba04ca03ea033a034a035a036a038a039a03aa03ba03da03ea040a041a042a2459c40ac2f806e10b97eb852bb64b45e"
    "af60aa60a2609a6093608c60866084609560a760ab60b360bc60ae60c860fb60f960f860ee60dd60c760ae6092627a5c"
    "66f750b63a9837a2449c59b873ff921c965e89684e803ea4489a61c676078a387e6a7e667e688066836286607c5865f7"
    "4dcc3b9a38a23fa252888f5ccf74eb5aef60eb60e660e360df60db60d660d260cd60c760c260b960a86090607d5e7e62"
    "9d66c668d866d660c95eb26096667c4064fb4dc43c9839a042a24aa04da04ba0479e459e45a247a2489e499a469a3da0"
    "32a2299c5ca04da03fa034a032a035a036a037a037a039a03aa03ba03ca03ea040a041a0429e43a82e8e54c4b27ebf54"
    "c662c360bc60b460ae60a760a06097608e6086608f60a260a660aa60bb60a760b260ed60ff60fe60fa60ee60dd60c360"
    "a460835c6b1f54eb409c34a0409e51a46de99a2989727b7e770a4a88439c58b06ff37f347a64742378437b607b4e7f6c"
    "837e763957ed439e399a3ca6488a6ed0ad76dc6cea5ee560de60d960d460cf60cc60c860c360be60b960b160a3608f60"
    "7f647d649135b133c25abe70b366a666936a7f646b2b55eb43ac3b983fa049a0519e539851aa4e9e4d9a4f9c4f984cb8"
    "47b43f96359a29945da04ea040a035a030a033a034a035a035a037a037a039a03aa03ba03ca03fa040a042a2319a3c90"
    "9b62c262a760a060ae60a660a7609260926092608a608b608b6098609b60a160b36089608c60c860f060da60ec60c960"
    "ab6095608862846c781a5cef49b2369c3aa0479660c88a137ff65ab6790f73c8529c519262c86407600558cb59ad5dd1"
    "62ef68f164c264d257ff48c03a9c38a43f9a54808402b864d062cc60c360bc60b660b060ac60a860a460a0609b609660"
    "8e6084627d647c4a85f194db9d01983a90468b4884367c4271365fff4dd842a6409e489e53a05ab25ce95ccf5ba45a9e"
    "58b452fb4aff42b838982c825da04ea041a036a031a031a032a033a034a035a036a037a037a038a039a03a9e3a9c3996"
    "399649aa72028748716261686564686267605f60606068607360816089609060946095608e607a607c609960ad60a060"
    "8d60636241643a644b5c663e72fe67d852bc40a83aa242a054a2658c6780638074917fc26fb25b9c509d47a5419f3b96"
    "3a883c873f813e813b8144804ba544c639b4329a3680468067ae8e16a854af66a968a064976290608b60876083608060"
    "7c627a66786879647a4a7a1179c5779f75a971d56ff8710474fc7604720e670358ed4cc446a64aa254b65fe1660569f7"
    "69ce66c260e158114f1d44ed3aac2f80"
)

CURVE = [
    [0.0, 76.0, 0.0],
    [0.0, 79.0, 0.0],
    [0.0, 84.0, 0.0],
    [0.0, 88.0, 1.0],
    [0.0, 91.0, 0.0],
    [0.0, 95.0, 0.0],
    [0.0, 100.0, 1.0],
    [0.0, 103.0, 0.0],
    [0.0, 108.0, 0.0],
    [0.0, 111.0, 0.0],
    [0.0, 116.0, 0.0],
    [0.0, 119.0, 0.0],
    [0.0, 124.0, 0.0],
    [0.0, 128.0, 0.0],
    [0.0, 131.0, 0.0],
    [0.0, 135.0, 0.0],
    [1.0, 140.0, 0.0],
    [2.0, 144.0, 0.0],
    [5.0, 147.0, 0.0],
    [10.0, 152.0, 0.0],
    [15.0, 156.0, 0.0],
    [21.0, 159.0, 0.0],
    [28.0, 164.0, 0.0],
    [36.0, 167.0, 0.0],
    [44.0, 171.0, 0.0],
    [52.0, 175.0, 1.0],
    [66.0, 179.0, 2.0],
    [71.0, 183.0, 2.0],
    [73.0, 188.0, 2.0],
    [73.0, 192.0, 1.0],
    [75.0, 196.0, 1.0],
    [81.0, 200.0, 0.0],
    [89.0, 204.0, 0.0],
    [95.0, 208.0, 0.0],
    [102.0, 211.0, 0.0],
    [110.0, 216.0, 0.0],
    [116.0, 220.0, 0.0],
    [124.0, 223.0, 3.0],
    [132.0, 227.0, 10.0],
    [140.0, 232.0, 19.0],
    [150.0, 236.0, 29.0],
    [161.0, 240.0, 42.0],
    [167.0, 244.0, 50.0],
    [172.0, 244.0, 54.0],
    [176.0, 244.0, 59.0],
    [180.0, 245.0, 64.0],
    [184.0, 245.0, 70.0],
    [189.0, 246.0, 76.0],
    [193.0, 246.0, 82.0],
    [198.0, 247.0, 87.0],
    [202.0, 247.0, 93.0],
    [205.0, 247.0, 99.0],
    [210.0, 247.0, 105.0],
    [214.0, 247.0, 110.0],
    [218.0, 247.0, 117.0],
    [222.0, 247.0, 122.0],
    [226.0, 247.0, 129.0],
    [229.0, 247.0, 136.0],
    [231.0, 247.0, 143.0],
    [235.0, 247.0, 150.0],
    [237.0, 247.0, 159.0],
    [238.0, 247.0, 166.0],
    [240.0, 247.0, 175.0],
    [241.0, 246.0, 184.0],
    [242.0, 246.0, 193.0],
    [242.0, 245.0, 202.0],
    [243.0, 245.0, 213.0],
    [243.0, 244.0, 223.0],
    [243.0, 245.0, 234.0],
    [245.0, 246.0, 236.0],
]


def rgb_to_hw10(r, g, b):
    y = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
    cb = (b / 255.0 - y) / 1.772
    cr = (r / 255.0 - y) / 1.402
    return 64 + y * 876, 512 + cr * 896, 512 + cb * 896


def boost(y10):
    yb = 64 + (y10 - 64) * 16.0 / 13.0
    ymax = 64 + (906 - 64) * 16.0 / 13.0
    if yb > 848:
        yb = 848 + (yb - 848) * (940 - 848) / (ymax - 848)
    return min(yb, 940.0)


def curve_at(t):
    import math
    d = [0.0]
    for i in range(1, len(CURVE)):
        d.append(d[-1] + math.sqrt(sum((CURVE[i][k] - CURVE[i - 1][k]) ** 2 for k in range(3))))
    s = t * d[-1]
    for i in range(1, len(CURVE)):
        if s <= d[i] or i == len(CURVE) - 1:
            f = 0.0 if d[i] == d[i - 1] else max(0.0, min(1.0, (s - d[i - 1]) / (d[i] - d[i - 1])))
            return [CURVE[i - 1][k] * (1 - f) + CURVE[i][k] * f for k in range(3)]


# Output tone curve: photo luma (8-bit, full range) -> drawn luma after the shell's 13/16 output scale
# (white ~777 = 8-bit 207, the HD HDMI bar ceiling).  The darks and mids stay at the photo's levels
# (slightly steeper through the mids for gloss); only the top is compressed, smoothly, into the cap.
TONE = ((44.6, 38.0), (100.0, 95.0), (165.0, 163.0), (210.0, 196.0), (245.5, 207.0))
TOP_CB, TOP_CR = -22.0, -3.0     # pale-lime floor for the top of the ramp (8-bit Cb / Cr)


def pchip(pts, x):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    n = len(xs); h = [xs[k + 1] - xs[k] for k in range(n - 1)]
    d = [(ys[k + 1] - ys[k]) / h[k] for k in range(n - 1)]
    m = [d[0]] + [0.0] * (n - 2) + [d[-1]]
    for k in range(1, n - 1):
        if d[k - 1] * d[k] > 0:
            w1 = 2 * h[k] + h[k - 1]; w2 = h[k] + 2 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])
    x = min(max(x, xs[0]), xs[-1])
    k = max(0, min(n - 2, max(i for i in range(n - 1) if xs[i] <= x)))
    t = (x - xs[k]) / h[k]
    return ((2 * t ** 3 - 3 * t ** 2 + 1) * ys[k] + (t ** 3 - 2 * t ** 2 + t) * h[k] * m[k] +
            (-2 * t ** 3 + 3 * t ** 2) * ys[k + 1] + (t ** 3 - t ** 2) * h[k] * m[k + 1])


def palette():
    """2 x 256 entries (Chartreuse, Ember) of 8-bit Y U V.  Each ramp colour keeps its photo chroma
    (scaled down only if it would leave the RGB gamut at the new luma); the top of the Chartreuse
    ramp is held at least at a pale-lime tint so the capped whites do not read grey."""
    ys, us, vs = [], [], []
    for pal in range(2):
        for i in range(256):
            r, g, b = curve_at(i / 255.0)
            if pal == 1:
                r, g, b = g, 0.62 * r, 0.55 * b
            y = 0.299 * r + 0.587 * g + 0.114 * b
            cb = (b - y) / 1.772; cr = (r - y) / 1.402
            yo = pchip(TONE, y)
            if pal == 0:
                w = min(1.0, max(0.0, (y - 225.0) / 12.0))
                cb = cb * (1 - w) + min(cb, TOP_CB) * w
                cr = cr * (1 - w) + min(cr, TOP_CR) * w
            k = 1.0
            for _ in range(30):
                rr = yo + 1.402 * cr * k; bb = yo + 1.772 * cb * k
                gg = (yo - 0.299 * rr - 0.114 * bb) / 0.587
                if min(rr, gg, bb) >= 0 and max(rr, gg, bb) <= 255:
                    break
                k *= 0.95
            cb *= k; cr *= k
            y10 = 64 + yo * 876 / 255 * 16 / 13          # pre-boost: the shell scales luma by 13/16
            ys.append(int(round(min(y10, 940) / 4.0)))
            us.append(max(1, min(255, int(round((512 + cr / 255 * 896) / 4.0)))))
            vs.append(max(1, min(255, int(round((512 + cb / 255 * 896) / 4.0)))))
    return ys, us, vs


def arr(name, typ, vals, w):
    rows = []
    for i in range(0, len(vals), 16):
        rows.append("        " + ", ".join('x"' + format(v & ((1 << w) - 1), "0" + str(w // 4) + "x") + '"' for v in vals[i:i + 16]))
    return "    constant " + name + " : " + typ + " := (\n" + ",\n".join(rows) + "\n    );"


def emit():
    h = "".join(KNOTS)
    words = [int(h[i:i + 4], 16) for i in range(0, len(h), 4)]
    assert len(words) == 28 * 122
    words = words + [0] * (3584 - len(words))
    ys, us, vs = palette()
    cs = [arr("C_FSA", "t_fsa", words[0:2048], 16), arr("C_FSB", "t_fsb", words[2048:3072], 16),
          arr("C_FSC", "t_fsc", words[3072:3584], 16),
          arr("C_PY", "t_pal", ys, 8), arr("C_PU", "t_pal", us, 8), arr("C_PV", "t_pal", vs, 8)]
    txt = ("-- chartreuse_pkg.vhd -- GENERATED by chartreuse.py; edit the generator, not this file.\n"
           "-- C_FS*: packed knot words {F8, S7, P1}, 28 rows x 122 columns.  C_P*: 2 x 256 palette.\n\n"
           "library ieee;\nuse ieee.std_logic_1164.all;\n\npackage chartreuse_pkg is\n"
           "    type t_fsa is array(0 to 2047) of std_logic_vector(15 downto 0);\n"
           "    type t_fsb is array(0 to 1023) of std_logic_vector(15 downto 0);\n"
           "    type t_fsc is array(0 to 511) of std_logic_vector(15 downto 0);\n"
           "    type t_pal is array(0 to 511) of std_logic_vector(7 downto 0);\n" +
           "\n".join(cs) + "\nend package chartreuse_pkg;\n")
    with open(os.path.join(HERE, "chartreuse_pkg.vhd"), "w") as fh:
        fh.write(txt)


if __name__ == "__main__":
    emit()
