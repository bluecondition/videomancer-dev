#!/usr/bin/env python3
# Copyright (C) 2026 czartroniks
# SPDX-License-Identifier: GPL-3.0-only
"""
RIPTIDE - build hook (pure Python; the builder runs it with the system python3).

Generates riptide_pkg.vhd from data fitted to the photo (WIN_20230309_00_28_35):

  C_RIP_KAE / KAO  4096 x 8 and C_RIP_KBE / KBO 4096 x 2: the swirl band's source field, a
                   256 x 32 knot grid in HSV polar form (hue H6, value V4) fitted to the photo's
                   top half-band (rows 327..519, mirrored about 519.5).  Knot c sits at x = 7.5 c,
                   knot row k at r = 192 k / 31 (r = 0 at the band edge, 192 at the axis).
                   KA = {H6, V4(3:2)}, KB = V4(1:0); even knot rows in ..E, odd in ..O, address
                   (row / 2) * 256 + col.  H and V are lerped (H on the shortest path).
  C_RIP_ZT / ZS    32-entry zone table of the texture over one 374.9-px column period:
                   mean hue, hue spread, red-bead threshold, mean / spread of the value, and which
                   noise mix (q, q + p, q - p) drives the hue (the zone's hue / value correlation).
  C_RIP_HB / VB    the crimson beads' hue and value offset (beads replace the hue wherever the
                   hue noise passes the zone's threshold, set from the photo's red fraction).
  C_RIP_COL        256 x 24: the colourizer, YUV of the full-saturation hue h (V = 1) in the
                   hardware convention (U = Cr, V = Cb), scaled x 64 per value unit.
  C_RIP_GSTRP      16-step table of the K6 Ridges control (ridge cycles per virtual line).
  C_RIP_ROWF/ROWC/SHF/SHC/ZPX, C_RIP_KS/KV/SC/SWSH, C_RIP_TFL: the texture lattice, ridge and floor constants.
"""
import math, os

HERE = os.path.dirname(os.path.abspath(__file__))

KNOTS_POLAR_B32 = [
    "t20v0f0f0f0fvsve0e0e0d0d0c09vp2jl0jijlllm6lplrldltisisjdjukekdjrg6cnbob9b9d7h7hoiojpkakdlejbjakai6c8ioitlak8kcjejfjvq6kskvmuve1n",
    "kdkfjvjfivifhvhvjcl7lbltkdlflvi6i5lfkuk9krlclekfjdkdlfhmd8b7c6ekj7j8ivivieidhdicjelpm5r2v5vn0a0nkrlvrt0f0f0f0f0e0s0q18265394akb4",
    "e3j6krlflflts50r16kml9ldknfgkcjsh7em5l392c1u1f1f0v0v0v0f0f0f0f0f0f0f0e0d0b08060k114gi1kil7lrluluiakbkdktkcjaipg6end9c9bqbqbpcoe7",
    "gmiojokdldlulvlvlvjrg8mckoieifivivpqk9kvkvte1akakvjvq7052i71b2f1k4l8ltlvldh4k9kalfkvkvkfjuithqf8cqb9apaoaoa6flirpo4buq0v0u0d0dvr",
    "tj0v0f0f0f0funuu0u0e0d0c0b0a14ijjdififiujtktltlulcitjdjfjvjvkfhmd6boapapaog5gnh7iokbkaktkuiqldh7cnbqf4nrlrllhahvhvodu3kfkvqd2fm4",
    "kfkfjvjfivifhvhvjdl8lrjthplulfh5j7kvkfkakslelflflflfg7a9aoc7dlg4j5jeivifhuishdirmsl8m5r3v6vo0avmmeotuv0f0f0f0f0d0c0q18255ja4b4b5",
    "dkimkrlflfltsm0s17lklalsiihkkvjchnd5472r2d1u1f0v0v0v0v0f0f0f0f0f0f0f0e0c0a08060j21b0hhkil7lrlulvjbkckdktkbjai8g6e8cpbqbabac9coen",
    "gmj7jtjfkflvlvlvlvirg6ltloieifivivrok8kfleuv0lkfkfoq0m0l1i81b2fhk4l9lulvlvimkrkslfkvkfkfjdish9eocqb9apapaob6f4qm3sn2os1e0u0t0d0d",
    "t30v0f0f0f0f0ooout0u0e0e0rvkhoivhvhfhfhfhfhvifkdmeisisjejvjvkfioe6c8b8apapemh7gmj8kakrkdlvjclvmfgnc9bre8kpm6rhs2vk0918m7jvse0qjc",
    "kfkfjvjeivifhvhviflolaitgoj8mfhlhllfkvkekrldlflflfkeb8aqa8a6blfji4jfkeksiehuitgdlbm8n3t20k0n0qv4mfoeuv0v0v0f0f0fv8ub1o264483a49k",
    "e3ilkbkvlflerl0b0qnikrmnfgkrkfjbi855392s2d1v1f0v0v0v0v0f0f0f0f0f0f0f0e0c0a0805031he1g1l3l7lrltlvjrjbkdktkbjai8fme8cpcabqbqc9coem",
    "h6iokbkcjdlvlvlvlvjreolrm9jkififkd1ok7kfoe1elljvos08070k2271c2g1k4l8ltlvlvj8kqlflfkvkfkfjeish9epcpb9apapao84372d3okumfvd0e0tv9vt",
    "t30v0f0f0f0f0skklfobu9u7l8iuivifhvhfhfhfhvhviehclsjtitjejvjvkfi8e6c8b8apapfmgndng6hmj6i6l8h5l7lql8g5emg8gsjdipiqt6v8vo06plud28o5",
    "nekemso7kmisidheifjtiuiuisi9kvi7gkjojpjajakskvlfktg4fjfmi8j8j7k9inirknlkjpivjpg9jvjshph7h8kmpnf8ntr9urv8rs0s0p0d27q6tm3472h4flfl",
    "i6jskfkelflur61a1cuknks470jrkvjui64m3b2d1u1v1f0v0v0v0v0f0f0f0btcvv0f0u0c0a08061310h0jhl3lolslulvkciqkuktjrj9hng6emc9bqaqbac9d8f6",
    "h6jokqlrkdlvlvlvmfkcepkql9nliiml0708qjjfqe0qirnfup0906041i8ha2g1kllplulvlvjpj6lvkvkvkfjvjeich9enbpbaapao7n482r2d2jifnevd0u1ev5q8",
    "t20v0u0f0f0f0f0mkckfkvjvjvjfivifhvhfhfhfhvhvifhtjbjsitjdjvjvkfi7elc8b8apaofm897p786n66655k63626161chsgrgmgk1ljf3k5jkfog6gbfuivmn",
    "mct7p2kelfnss7rlkakfjvjfjfjfjfjvleldlcp50mu6olknlclslektkukekckdkri44k5k4jg1h6l9ehhdhfgvfvffffgfgvgegehfhtiqioj9iokbkckskrkvkvkf",
    "jvjvkfkfkfkvkslor70q1r1r1p1lf2knlfksnn2p2c1u1f1f0v0v0v0f0f0f0f0evtt8qaucvr090mt2j4j8iqirjakqldlvlvlej8ldlfkvkukdjchph7hoepbqbab9",
    "b8c6dlh4k6kojqkfkelemtktlcmpjphngcgrppvauouco5mfvc0c0a1n13igi1h1h1jkkpkulflvlcikknlelfkvkfkfjvjcgoe8c98o67493b2d2euotaolnruu0v0d",
    "t20v0v0v0f0f0f0f19lmkfkfkfjvjfivifhvhfhfhvhvhvhthshqibkcjtjvkfkfh6e7cob9baaq8l6o685n574n46463l35442ij0j1j3jjl1rgngngrgrgnhjhe4gf",
    "hvhuiujejvjvkfkvlflfkvkfjvjvioimjnktlfkvmov80d1qm2laltlflfkvjqhlej3l3m28282514f9r9jgifk9q0j0h7gvgfgfgvgvgvhfhvifivjfjvkfkvktk8j7",
    "imhnhni7j9kckukukqnos3qm1a19274jkllfkvnosp1t1f0v0v0v0f0f0f0f0f0f0f0vvohrlfktkcjtjvjfifhvhvifivkelflvldhklelfkvkvkfjvjdirjrkah8bp",
    "apaoanamcjhlilivifhvifhthdhum8n4q2u4070a0b0atkmevd0e17knkdjvjfiuiuiuivjfjfjblumcsioilalflfkvkfkfjeh8en885n3q3c2d1u2ttt4fk6nfvv0f",
    "si0v1ss8vf0f0f0f0f0tumk9kfjfivivifhvhfhvhvififiehbhqibksjtjukfkfi8emcobpbpaa8n6n5o4o48483n3m2m3kr2imjtjfjvjtk7l2n1m0mhj1phviu2m0",
    "htifivivjfkfkfkvlfkvkvkfkfika3e3ilkqlflvles5vr0f2ll6lrlvlflfktj8h574452n282l1mh6r8g3kft1v41jtgfghfgfgvgvgvhfhvifivjfjtjoik2k4l4k",
    "7l9lama6dkhmkbkvlelcqlmluo1q2n45h3lvlfoat91e1f0v0v0v0v0f0f0f0f0f0f0f0f0lkekvkfjvjfivifififififiviclelvh4lblflfkvkfjvjeicg8dobpap",
    "apaoanamb5gki5ivivjehvhvhsgvidmmqiv5vn0a0a0bnkodvd1oknkfjvjvjfivifivivjfjfirh5n6sjo5l9lflfkvktjvjeiafn78583r2s2d1u1tsmoplemvve0f",
    "031f1njurt0v0f0f0f0f0f0bvpq7n9jpj9i8i9ipjpkqldldjuibibkskcjukfkfi8end8c9bp9q876o5o4o4o483n374jiojfkfjvjfivivivjqkkn1p0rgq0u1v32j",
    "igieivjfjfjvkfkvkvkvkvkvjoaib5d4i5k9lelflfr6vrvu0ql3lblulflfktj8gk5k452o281n1ki9oojep72k13vjvjuhfhhfgfgvgvhfidi9ilijih3j343m5575",
    "96anb7b7cmh6kbkvlflepnmnr61q2n45j3lvlfp9tq1e1f0v0v0v0v0f0f0f0f0f0f0f0f0fvmkakfjvjfjfivifhvififivichomdg4kqlfkvkvkfjvjeisgpe9cqba",
    "apaoaoanb5g3i7ifksmojqhfhthehunlqiulvn0a0b0rl5pt1trijfmtp6olhmiqivivjfjfjvkuj9pkr4mmmolekvkvkajvjehpf75n493b2s2d1u1umllvlvmvvu0f",
    "v31vn2kfkete0f0v0v0f0f0e0d0q0n0k0hugogngm1lmlqlumekcirjskbjukfkeh7fmd7bp9p8a775n4o493p3o3n53r4r4sjj3jbjfifhvhvgvhviah6gkgio1u1ti",
    "2km1h7ivjfjvkfkvlflflflfhi8kbkakh4kplelvlfpo1q1d1e54l8lvlflflfkai5fj3l3l171q3ljqj9js02lih7r3324hv0lgh5gmi5j1lgt03g2i233k3k3l5465",
    "7594alalbmg5jqkvlflfmaj3om182n3lbhldlfp8ta1e1f1f1f0v0v0v0f0f0f0f0f0f0f0f0f1qt5ono8j6imi8iqjciviviuh8kqi5lslvlflfkvkfjvith9eoc8b8",
    "aoanamakajg1gnivl7kom9jbhfhtgfirpj04vo0o0avniardv7kbqs28173l1h80g1hmj7kpltlvmenlskojnml7lfl8k7kfkeiqgm4o4a3r2s2d1u2fo3kvlekuuf0f",
    "0j1v1dudusuf0anbnsucvevv0e0e0d0b0906131iqhjil4l6lalslelelvkakclfkukdjriohph8f8dp887857493a2r2r2b1s1q1nurubu9h9ldsnu9u8uounv4vkvj",
    "ui1i10jgififififjvkdjfkbh4fl95gkglg4inhnjpkan8nbnamrknltkqjnj5f5bmbme6h8gps4vls4unvd0utruvuaoapomrnrus0c0b090o16153352cid2iil6la",
    "lblululrnmkaj8h4eihmjtiqgn684p4a3b3b2r2d1t1f1onmtr0u0e0d0d0c0c0b0b0a080604s2pirhqhk3knkbjuivjujfksjvjsi7gmdmc6cnc8gmjqjqhoj8jpkq",
    "kpldltlululelflflflelqr3u3vl07smn8rqvmm9sruniqjvgfhuhlikh4iniqkskukui8h6i7kajqipjpiqfnfng6f7e8i9896o674n3a39391p1pvavq0a0a09vq0a",
    "t20v0v0v0f1vn3kvlfkvmevv0f0f0f0d0b090714210gf0khl5lqlvlvlvlbjklvlelfldkokfjuiafnd86o573a2s2d1t1f0v0v0f0v0u0f15rp0s0b0b0a090p1ku2",
    "rgq0h1nihphfhvhvifjcivi4c4ala69n76j7boa8anf4jcjukfkvktldmegi4l86a7aob9enl9jlrjvm0d0f0fvfvfqlsv1rldkvpu0f0f0f0c0a171l5ia3b4ajh3ko",
    "lelflvlfq5p3s50l0jjjkvjviq5n594a3r2t2d1u1f1f28kflftt0v0f0f0f0f0e0d0c0a08060j2hcgihl3lnlplriuitiukajejvh5d6bob9apaqaq9nh6hpi9jrku",
    "kclflflflflvlvlvlvltl8lls3v6091np5rtuondvv1bj6jfmbtivh8g70khlklplulvktj9j9lvkckbkdkeiqgoe9cq8a6q6p594a3r2r2d1t1t1c1c1b0q0q090808",
    "vk1fvtvv0f19ldlvlvlvlfpd1f0f0f0d0b0n0kr0i0gifiilkqldlvlvlvlaj5lvkqk6jjjmjvjdhpfn6o5n4a3a2s2c1t1e1e0v0u0e0e0e0on9tr0b0a0a0pviimja",
    "llo1jgmjj6hvhvhvifjsivind3am9l96c5job7aobng6jakdkvlflcldluhi7596a7b8bmm7lamku5vr0f0f0fvfvuqbvf0vn6kvoevv0f0e0c0p0n25338i7ihhijko",
    "ldlvlvleonqjr50m33j4kvjvi95n594a3b2s2d1u1f1vt4lflvleud0v0f0f0e0e0d0c0a080603v1ugjhl2lmlalslbjdjekcjsjvi7elcnbob9aqaqb9g5hpi8jakc",
    "krkulvlvlvlvlvlvlvlulpnjsi1lr0oosmsbsmod0f05ifpq0l0l1341c1hhkkl9ltlvluksiplfkcjakbjtiagoeocq8a7q6p5o4a3q2r2c1t1t1t1c0s0r0b0a0q0o",
    "071cuu0f0vrjlflvlvlvlvlfntusvrvpt6k9iriuivififififivislulvk8iklvk6h2iijpjvjdh8e868583q3a2s2c1t1e0v0v0u0e0e0e0ol7rq0b0b1ooihuifhf",
    "hvl5n1kjm4idhvififjrivipcj8lcl76jlg6aoaobng6ipkdkvlflclemc73758mb79nl5lukoq3vpvv0f0f0fvvvqqd0f0um8lflvvf0f0e0c0a181je1glhqjejfjf",
    "jvjslvltq5s3t61m1jjnkvjvhp5o594a3b2s2d1u1f1ul4lvlvlvleut0f0f0e0e0d0c0a080604uis0hgl2lnlplslsjrjekcjsjviof5cnbob9aqaqapflhph8jakc",
    "ksktlvlvlvlvlvlflvlskajaj8j8ivodvouto5qd1vn3os0807vl1i41c1hhkkl8ltlvlvlfjakuktjqkbjdiagoeocq7q7a6p5p4a3q2r2d2s1q0r1a0p0q0o08rkmp",
    "visovf0f0tkqlflflvlvlvlvlflflekejvjvjfivififififivivhrj8mfilikluijdij2kfjujch87868584a3b2s2d1t1e0v0v0u0e0d0e09j8nuun0mjmivifhvhf",
    "gviqo1kjm5iqhvififjpiujvh3h5f6cklbc7aoaoc7g6jakekvlfkrlfm85k759m8nk4lvlcm7rlvevv0f0f0f0f0bpu0f0slblvlfrc1f0e0c1pt3i8jvjfivivjfjf",
    "juiqkqmcqku4sm1m11kbkfjvho5o594a3c2s2d1u1f1sl8lvlvlvlvleut0v0f0e0d0d0b0o151i21vgi1l2lmlqlsltkbjekdjrjvj9g5d7c8b9aqaqaqelhph8jakb",
    "kskclvlflvlflflui8fohehvifivifrp0bvsm7sf1bn8tr09070l0j4hbhghkkl8ltlvlflvkbktktj9jrjdirh9eocq8a7a6p5p4a3r2s3rn3kvleldldldldkukfjv",
    "q0rs0f1fs3lfptmslflvlvlvlflfkvkvkfjvjfivififififiviuich6mrfjillthjh0kakfjuiag76o5o4p3q3b2s2d1t1e0v1f0u0e0d0e0ojokvlrkbjfivifhvhf",
    "gvhfmjlkl6lliuififjoitjfjtkvejj6j8amb8anc7g6jakekvlfkqlfm7648666iilulvlcpmurvf0f0f0fvf0vumpe0f0blrlvlvlfqb0bunk7jvkfjfivivivjfjf",
    "jujainn7tltlt626jgkvkfjuh858593q3c3b1s1u1f2pldlflvlvlvlflesb0e0e0u0b0molipipimk2n0kilmlqlsltlcjsjukajvjag5e7cob9aqaqbacmi9h7ipjq",
    "kskclflvlvlvltf4caeagshvififp8vpvs0cl6svvnpqvs0q05032251ahghkjl8lslvlflvktkblfjojsjtirhpf8bq7q7a695o4a3q3c33kvlvlvlvlvlvlflfkvkf",
    "s10f1ft4ifpus8melflflvlvlflfkvkfjvjfjfivififififiviuhpflfkc3l7kph2k8kfkfjcgn7p6p5o4p3q3b2s1t1e1f1anntt0u0d0e0lkdkfjfjfivififhvhf",
    "gvgfknlkl6lmk6ifhvjogijvjah4h5k9a5aoanancmh6jskvlflfk8luoj653kk3lvlflfnotqvf0f0v0fvvvv1em8tv0f1fm9lflflvlflekvkvkfjvjfivivivjfjf",
    "jujahls438v5t63ljjkvkfjdg7584p3r4b1m1t1u1vu5kvndlflflvlvlvkvmtpqp8ksjuivivifhvhfibl4lmlqlslultksjukrjvjsh6emcobpaqaqaqc6iah8i8jq",
    "ktjrkvkujsj9e5a9caeagrjcj7sj06vp0c0rl7tto5sd1so3iejfj440ehehjjl7lslvlvlvlejolfjpkejujchqg88r8a7a6p5p4q3b4akklflvlvlvlflflflfkvkf",
    "tivf0eurrpspqomslumelelflfkvkfkfjvjfivifhvhvififiuichpg6emgkk9joj6jqjbirgneo786o584p3q2r2c2c1r16nmldq8vavr0cukjskfkdjuivhvhvhfgv",
    "gvgvk8lklkm4lki9jfi5fij7g4ckglflc6a6d6dmgljqkbktksnao6n8tjhkj5ltlflunas8vevv0v0f0f0fuuvuucvfvuubpnlululflflfkvkvkfjvjfivivivjfjf",
    "jciohk5k2lskulqjkbkfjui9e7494q482pqmut2d0br7qpubs6ltlulvlvlfkvkfkfjvjvjfivifhvhfhfiulalqlsltldkdjskbjsj9h6e6d8bpb9bpc9cognh8j9kc",
    "kbktktkvkdkciqioenj9jslpo5u5vo0a0c08opq9oavfv3ivkfivksq18hk1lkl8lclulelfkukcktjqkcjbiah8e88q8a6p5p584a3q3llqlultltldlvmdlululuke",
    "shvevf0evuussqtpsmp6kulfkvkvkfjvjfivivifhvifififieichqh8hmhljpj7j9injaiagneo796p5o493q2r38trpllbntnpnrpkur0b0oqiksk8k9kpjcieguhf",
    "gvhfiqm4m3m4mkj4hpj9i3e4h4hkd4g6b6d6e6e6gljrjqlck9nrnnomollpldkrlumtopspvdvv0f0f0f0tuevuvuue0fvcsor6lclvlflfkvkfkfjvjfivivjfjfjf",
    "jdiphl14rkt5sjo6kakejui9f74o7mnmnbq7tr2ctrubtsvv1f1bq4ktlvlflflfkvkfjvjfivifhvhfhfhfhvlclsltltlejtkbkcjaiogme6cobpb9d9c9f7fmi8kd",
    "kbkskulekukcjahnioktkdlnq2s2tkv7vrvnnoobqbvsmnkfkfjvketj041ipik4k6k6ktkvkekdkdkbkbjaln7k556688hmg75o59492mm8r8tq2c0oo6nrr7q8qnnp",
    "vk2fvntf0f0v0v0v0f1uo5kvlvlflfkvkfjvjfivifhvhvifiviuirg6dlblghmeijkukfkfisgo7q7968594a3b2s2d3cn3lflflfmvvs0f0qjmkvkfjvleu506qhg9",
    "hvgviemkl6lnl8mmipifk6i8jvjth6hmh6ema8aoanhklvkvkvkvlfkqlvfih1mflvlfmfq8uuvf0f0f0f0f0f1fq7uv0f0f0f1fumkelvlflflfkvkfjvjvjchniojr",
    "kekejapkv71p0m17p1kulfkfi8583qhklvkvqr1u1f1nps1f0f0f0v1ql8lflvlflfkvkfjvjvjfivifhvhvhvhvldlulvlvlejcldjskejsi8g6e8cpbqbqb9boflkb",
    "jak8lakbitleidmflvlvlvkchuiciehftb0vo8ufvv2fonkvlvmflfpu2s1a474j3ichjejfjfjvkfkcjmt3vpvcvr0ro1kvlvkvqn584r4d4mjetu2f3dsq0f0v1f0v",
    "031v1rte0v0f0f0f0f0f0vvnl9lekvkvkfkfkbjoiminiriuiviviui9gldk7kk4j7k8kvkfjvhqen6a69594a3r2s2c1u2mjunflujvpd0u0vvkkekvjuivqq081ouh",
    "fohfgvl5l4lnlpl9m7itjfjljfi7f4e69nflf6b8b9b8i4ltldldlfkuktmdfkl6lflflfnut9ve0u0u0u0e0e0vv5sd0e0e0e0e1u27lplvlvlfkvkfkfjjf083c2ij",
    "knlulfmpr31o1qvp28l2kfkfjdg65a03kukvrc2t1f2cq8uv0v0v0f0f2clmlflflflfkvkfjvjfivifhvhfhvhviflulvlflvkckclfjrjsjtiagnf7d8bqbab9b987",
    "im6m6436t68jfib4mkm5n3e5gtgeifipnmuiu3o5p5oqqko5kqkfnpn8msn9kajqjokajvjvjvkfkfkfksjormv9vq1b28mklrmsnbposq3asllepauuvfvquv1fvf0f",
    "qhvf1f1dte0v0f0f0f0f0f0f0v0ssnmsp9rnu70m134hc1ighhj5jtjejahlg34433jjk4kvkvkfjcg75p5o4q3r3c2s2d1v38lbsrtnkamdve0fvnsa0ss4hunstn08",
    "v3e0hvgvl4l4l8lslclrksiukajuinfkdmaob8apc7d88a7p6p8nlqlvlukvkrkbr2o1p1j3jfiuiehuhugugugegfgfgegegehehuifjdjejvkfnfr5um3l2k6ia4a4",
    "c3i3k7lcldlqqlv80s1s1nnkkqkdoarnsloljslbts1u0uussauv0v0f0f0v0qmolelflflfkvkfjvjfivivifhvifivldlulvlvlek8lfktj6jpkfjdirh8eoc9b9a9",
    "77483b2s2d1e0u0t0t2t2bvq0q0708ulo5qmpjfthfgvgtguk6l2l2kjl4k4k2j5ioghh2imimh5imkajrjqfnf6i8hm0h8272bhsjj6jsjcoptmtnqjhnmqucvcvata",
    "sgue0f0f1es9uf0f0f0f0f0f0f0f0v0e0d0dvs0a070l22a293g1jjl8ldkdj62l1o2726qikqlfkfkc664o493q2s2d1t1e1f2mnavf2fo5luuv0vvdvv0v1mjensu7",
    "08v3lghcifm2i8hfjelvluksjckbjukai7g6dncob99a8q8a7a6p5o5nn7oavbulktlvlfmfmrkvkvkfjvjfifififhfgvgvgfgfgfgvgfifh5einjqjnk4j3k354k74",
    "9la5a5e3hljqkukvmoqjsn1q182mkmlvncspur2pl6lerc0v0f1cqavf0f0f0f0v1domlflvlflfkvkvkfjvjfivifififksltlvlvlvnmkomoggj2kvkfkfjdh8f789",
    "58593b2s2d1u1f0v1fsklsuv0f0f0e0e0uummb17ipivifhfgvhfgvkslqkdivlclbkbjfjbfkcmbn6mj47o797q656p7q5p493q3q3938372n27261m1413vjrhdgh0",
    "tj1f0v0vumsv0v0f0f0f0f0f0f0f0f0f0f0e0c0907153i92aie2k4l9ltldi62l271n0mphlclfkfiq5n594a3r2s2d2d1e2eo4uf0v0fvlosvv0uusvv1uk5kfu90q",
    "0604bghuirnhhqiflulululcjckcjuj9hnf6d7c8a99a8q7p7a686q3jnq3q2d5olplvlfmumdkvkfjvjfivjolkk4ishfgvgvgvgvgvidl2t0v122233j252l364k74",
    "a5b5b6bmgkjpkulfmdpkt619192ol5mfoqro1e49krmuvf0v1fs6tv0v0f0f0f0f0vmnlflvlflfkvkvkfjvjfivifivivkdltlvlflvk8k7m9egjlkvkfjvjeh8d92n",
    "glo82r2c2d1u1f1v15iflfuu0f0e0e0f0ttltqiljfififhvhfgfielolniuhviulajrjfjuh4d5b7dn74797p897m796p6858493a2r2q2a1p1p180n0n060l15uk13",
    "t20f0v0rsc0f0v0v0e0v0f0f0f0f0f0f0f0e0c090n1442a2bihik5lalflfnnrl281m26ohktkvjui7584p3q3r2s2c1t1v0mte0v0f0vo4tv0v0buf1fr3jvub1av7",
    "vmvjg2hfk6k1islultltltlbjdkcjtj8g6dncoao8q8p897q6p6q452n3q2b2ct3lflfmdn9kvkvkfjvkdsj2j3311h1hvgfgfgfgvh8o1t1vi0j13141k2526364k6k",
    "9lb6amd4i6kblflfmqrkva1a0q3mmpntqo1s2urjldve0v0utruv0v0f0f0f0f1fullflvlflflfkvkfjvjfivififififktlvlvlvlvj5lvjmi0kvkfkfjuh8do3574",
    "4m3r3c2d1t1f3cn5ifobvd0f0e0d0e0bunqmjcivififhvgvgvi7mjm5ithvifiujdjbjfjvjahml87m486n6n7779796p5o4o49483o37372m25251k1313v2uh01r0",
    "rhuv0vuruv0vu9pokrosvf0f0f0f0f0f0f0d0b09070412212h91lkl8lsluml452m1mqjnildkvjq5n4o59493a2r2d1u1psc0v0f0f0trpvf0uubvfvlkduc19pmk9",
    "qn1hgsien0hmlqlrlrkslekbjujukej7fldmbnc8bo8o7p797a4l484a3a2r4lkaler82mmkkfjtkfomqio2nioioggagfgegfgam0vgv1viu2sju4ulvm162l355kck",
    "d4a4bkilk8lclelerjv90r0q1p3jooro2r2c3no80e1f1cua0f1f0f0v0u0e0uu6iflvlflfkvkvkukajsjfifififivivjekulvlvjnj6lvjlkukfjvjeg7d87a7a4q",
    "5a3q2r2c1t29k9lvuc2f1f0e0d0cvpv8qlgujfifhugvhfjso0ljk6hchfhvhvjfiviogjg3fjf456slim5m797p897p7p7978766l645j63423i3121111100v1v1u1",
    "sj1fvtvv0vvvveuuuruf0f0f0f0f0f0f0f0e0f0f0f0f0f0u1c19465383a3a5d5h4j9kelfn8qjt60p1a04k7lfnrr81s1u1f1f0v0v0v0vtpsu0v0vvtud0f0f0v0t",
    "0evbs4sl2400hghhi5ipifjvmflek9mfjljmjmlfkvkfja8o8a7a6p594a3r3c2p1t1v1f1f0v3cqikvlflflfkvss0c0b0b0a090o090807vmu5rko3iih1i6j6kmj9",
    "lrlqmok67584hke3jma8776o685p493q2q1q1c2q1c0pvr0r0cvmsjs3s3u7slnqgseafbfagvgvhfhvifiejejujvjvjvkfkfkekfjtjrhli7kpkbkrn7ijsjqor636",
    "ujnljpmanaopvnu7vrtcsau8urujobtqtpr7kfto1mojmeppv5j8jvnokok3g1gjiujfjfjfiqk9lajnldlo0185685o58574o494a3b3b1a2d1u1e0u0v0v0fvvvevf",
    "tj0eusvetqtv0vvuuevv0f0f0f0f0v0v0e0f0f0f0f0f0e0s1a28356k95alc5f4j7kslemcnosmt8191momnpnbqoub1s1d1f0v0v0vvdvvvcvf0vve0fuv0f0f0fvc",
    "tr0aq3kcjai7inh5jeitivjtkuj9h5k8jpk9kakejuibg88q7q6p69594a3q2q1s1u1e1f0v0vu7lblvlvlflvqo0d0f0f0f0f0f0f0f0f0f0d0q0n16181b1c1s2b2p",
    "475l6l8nb7bnbndnfn8n7n77767i5j5k52lmsq1d0u1bq7sruvvu0f0tpmktpau90sv9v8umrkojjbjfjeiahrhbgthbieibiqjqkrkcinh6inglgkg7dmdp675lmmm9",
    "nbr5v7rjm7mrlalan8mom9mqichtgbgbfbdtgeevhtgdfdifivp4kmjbivkekuleldkujqhlekd4ilk9k8lcmco50n0q18273k4k3o3p2r3qs71d1d1d0u0f0f0f0f0f",
    "ujvtuuvuvvvuvf0f0f0f0fvv0f0uunnpmdnatevv0f0f0d0c0p1m336ibjbkdjk5lalvlvnpu50r1t1c3pqimtr8vr4c1t1f1f0v0v0ut9uv0f0v0vusuv0v0f0f0f0s",
    "t7pkivjfifkaisjfifififisiaf5e4fjkqktkdkfjcgn997a7q7a69594a3p2b2d1t1e0v0v1ckllflflfkvra2d0f0f0f0f0v0v0v0f0f0e0b0a0b0f0f0v0v1f1e1s",
    "2b38557m8n877878796p6858493p3p383735351j130i0h0g0g0000vg00vg10s0i0o0l0f8ifhfivjfkfkekundqmk3lukukejvh5g3j6lrlvlvlfnqtorjsm0953m8",
    "mfmcq8u91b2c1t3f1f1e2f1qvr1t0c0qu9uu0suqtooqtajbnt070n1luiegichvhuhvivicj8g5fji4hljofnf8a99q9a8a6q594a3r2s2d2e1u1f1uusuu0f0e0f0e",
    "u30f0f0sq8uvvv0f0f0f0f0vunlclelfmvud0f0f0f0f0d0b0p0o17253k5j6jbki4jnk8jno7npsntnplo5onpmolpbt8291b0t0v0u0v0v0uuevvvevv0fvf0f0eso",
    "q7rqjmjejticieiehrivjeiciqlcjahmi6jnkaj7jai8hph9797969594a3q2r2c1t1e1e0v0uvspomcldlvmtocvcvvvvubratuuuvv0f0f0e0e0d0d0u0u0t1c1s1s",
    "2c2r3a3p4o57687879796o5o4o3p3a2b1s1d0u0u0v0f0f0f0f0f0f0e0e0d0d0c0b0a08vl52kif6hfgvhfhfhfhvififjfjfkejqinfjb3mnno467o77867976m4e5",
    "gjb3lmipjah3c4mhgqhfidgdhshtgvgugvfvkbk9lcr6qmr4sps7nstv0fvd2arnsopmlclvlcikc5blbla597889898b8b7cmb5733l291r1a0cuauu0v0f0f0f0f0f",
    "u30f0f0vsouf0v0f0f0f0f19ldlvlflvkvmuvv0f0f0f0d0c0t0u0u1d1d1r2o3m5l969ma6b6dlg6j8krkbkukejsh8lslaikkoldoqrarptvt8tfuuo9vf0f0f0dtf",
    "0vuv1dtota1prhr5l0isivjpkikokdjthlj5rl2m2l2iknkfio6o6p5p4p3q3b2s2c1u1f0v0v0v0v0v15kulflfkfst0v0amcnenalsuv0v0f0f0d0c0s0t1su8sq1q",
    "1q2a3a3q495o6o7879796o5o4o3p3a2b1s1c1d0u0v0f0f0f0f0f0f0f0f0f0f0f0f0f0d0b090m0ik0hrhfhvhvifiehshribirkakckcinh6cm7p7o7o6m79695627",
    "581ko82p37uot5rnvkoih6jvjfkahvhfhfiegfevhfgvfvhfpjv1o1lhlihdjvkvkulfonf1a32m3o47566m97anb8b8b8d79n56482p2b1s1qrotrvv0f0fvtuuvuvd",
    "u30f0f0fvruv0f0f0f0f0vnllflvntmtlvlfsu0f0f0f0f0f0f0u0t1d1s2b2p476596b7anb7cmg4imk9kpkclektjrh7memfotsqveuuutvf0fvf0frmtf0v0f0vsr",
    "vvuquf0v0cvr1nokrkggjfivjtknlslvldpkqk1n161mk3kfiq6o695o4p3q3b2s2d1t1e0v0v0f0f0f0sl9lvlvlfst0vt8psvv1fupsu0f0f0f0e0d0u1evptt1u1t",
    "2c2r3a3p4958686o79796o5o4o3p2q2b1s1d1d0u0v0f0f0f0f0f0f0f0f0f0f0f0f0f0f0d0b080723k0h8hvhvififidhah9gohokakdkcjqi68n7p897p796p6957",
    "2p3b37sp1r1q1q1p0p0o1kj2jfkfq3flifhfjciridl4htgvgvkkriul05vijpkfkvkvkfj664744m3o484o677o98a9apa89866483a291b1d1u1f0u0f0f0f0ermms",
    "u40f0f0f0duv0f0f0f0f0pldmfrt0ut7mtlfru0f0f0f0f0f0tv9s8tr1c1r2a3n5l86a7ananb7dlgkj7kpjqkvkckuiaellcoctcvf0f0v0v0fvv0vuotf0v0f0vtq",
    "uv1tq8vf0c0b09u1m76ghnjfivkelclflvmppl27151mkikvirg75o5o4p3q3b2s2d1t1e0v0v0v0v0f0ul7lflvlfrs1fqmtf0fvv1fudvv0f0f0e0d0u1cus1e1d1s",
    "2c2r3a3q4958686o79796o5o4o49392b1s1c1d0u0v0f0f0f0f0f0f0f0f0f0f0f0f0f0f0e0c0a07vl3jkgiaifififieicgognflj8kbkckujai688897q7q6p5o5p",
    "482p2d3p1b1c1b0q0p0a0825j6jfppvij2iugfjqm4lliihfgvhtnntbvv0f3qklkvkvkfkvj2bk4k4646486l6m8o8og4987n56482p2b1t1rtpvd0u0e0e0f0qqprq"
]
ALPH = "0123456789abcdefghijklmnopqrstuv"

# photo zone statistics of the base texture, red beads excluded (zonehsv_b.py): hue mean (turns), hue sd (turns),
# V mean, V sd, corr(hue dev, V)
ZONEHSV = [
    (0.5996, 0.0698, 119.3, 69.7, 0.102),
    (0.6073, 0.0650, 129.2, 67.4, -0.259),
    (0.6090, 0.0597, 134.2, 65.2, -0.491),
    (0.6138, 0.0602, 135.5, 61.7, -0.589),
    (0.6166, 0.0634, 136.0, 59.7, -0.580),
    (0.6222, 0.0715, 131.9, 57.1, -0.549),
    (0.6231, 0.0764, 130.4, 57.3, -0.523),
    (0.6267, 0.0781, 130.8, 56.7, -0.487),
    (0.6245, 0.0779, 125.3, 56.5, -0.492),
    (0.6197, 0.0744, 126.3, 58.3, -0.502),
    (0.6156, 0.0743, 120.8, 59.8, -0.553),
    (0.6121, 0.0707, 113.7, 60.5, -0.598),
    (0.6069, 0.0667, 115.1, 65.0, -0.616),
    (0.6046, 0.0687, 114.6, 68.7, -0.507),
    (0.6014, 0.0860, 113.8, 70.9, -0.161),
    (0.5936, 0.0836, 117.9, 70.2, 0.411),
    (0.5882, 0.0751, 125.1, 65.1, 0.686),
    (0.5856, 0.0733, 132.4, 58.8, 0.766),
    (0.5832, 0.0708, 137.2, 51.6, 0.796),
    (0.5831, 0.0698, 139.1, 45.1, 0.739),
    (0.5863, 0.0697, 136.3, 41.5, 0.507),
    (0.5953, 0.0736, 131.1, 40.7, 0.227),
    (0.6070, 0.0794, 125.3, 40.5, -0.005),
    (0.6157, 0.0776, 125.2, 41.1, -0.166),
    (0.6151, 0.0770, 123.6, 44.1, -0.185),
    (0.6087, 0.0763, 118.8, 47.7, -0.180),
    (0.5991, 0.0717, 116.0, 51.4, 0.036),
    (0.5923, 0.0619, 122.5, 51.2, 0.473),
    (0.5888, 0.0608, 127.4, 49.7, 0.713),
    (0.5872, 0.0614, 128.2, 51.9, 0.754),
    (0.5901, 0.0601, 128.7, 57.7, 0.671),
    (0.5959, 0.0631, 125.8, 64.2, 0.431)
]

# photo red-bead fraction per zone (hue 290..30 deg, V > 40; zonered.npy)
ZRED = [0.019, 0.065, 0.115, 0.155, 0.158, 0.204, 0.240, 0.272, 0.263, 0.242, 0.240, 0.226, 0.186, 0.112, 0.042, 0.003,
        0.000, 0.000, 0.000, 0.000, 0.000, 0.003, 0.025, 0.070, 0.083, 0.049, 0.005, 0.001, 0.000, 0.000, 0.000, 0.000]
# P(q > 2 T + 1) of the hardware hue noise q for T = -32..31 (sampled from the lattice hash)
QTAIL = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.9999, 0.9996, 0.9993, 0.9986, 0.9973, 0.9952, 0.9923, 0.9884, 0.9832, 0.9765,
         0.9672, 0.9557, 0.9432, 0.9258, 0.906, 0.8823, 0.8563, 0.8266, 0.7938, 0.7589, 0.722, 0.6789, 0.6344, 0.5919, 0.5481, 0.4998,
         0.4532, 0.4054, 0.3625, 0.3222, 0.2803, 0.2389, 0.205, 0.171, 0.1403, 0.1144, 0.0913, 0.0726, 0.0552, 0.0412, 0.0304, 0.0222,
         0.0159, 0.0114, 0.0073, 0.0043, 0.0019, 0.001, 0.0005, 0.0002, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# texture parameters, tuned against the photo (tune12.py, texgen12.py; seed 94, sw quantized to 1/8)
TEX = {'fLy': 2.3876, 'Ly': 8.0, 'shear': 0.061, 'ns': 2.3807, 'gH': 0.4367, 'gV': 0.8538, 'mV': 1.0172, 'dV': 1.887, 'dH': 3.4937, 'dith': 0.0, 'Pv': 10.8, 'ks': -0.1167, 'kv': -5.5492, 'fl': 40.0, 'bsc': 1.6707, 'hb': 332.5904, 'vb': -44.487, 'rth': 0.2207, 'sc': -0.3909, 'sw': 0.125}

ZPX_K = 44747       # zone phase per virtual px, Q24 (5.121 periods / 1920 px)

def lattice_consts():
    """fill in the lattice constants from TEX (Ly, fLy, shear)"""
    g = {}
    g['ROWC'] = int(round(65536 / TEX['Ly'])); g['ROWF'] = int(round(65536 / TEX['fLy']))
    g['STRP'] = int(round(65536 / TEX['Pv']))
    g['SHC'] = int(round(65536 * 16 * TEX['shear'] / TEX['Ly'])); g['SHF'] = int(round(65536 * 8 * TEX['shear'] / TEX['fLy']))
    g['ZPX'] = ZPX_K
    return g

def knots():
    s = ''.join(KNOTS_POLAR_B32)
    out = []
    for r in range(32):
        row = []
        for c in range(256):
            i = 2 * (r * 256 + c)
            v = (ALPH.index(s[i]) << 5) | ALPH.index(s[i + 1])
            row.append((v >> 4, v & 15))
        out.append(row)
    return out

def grain16():
    """K6 Ridges: ridge (stripe) cycles per virtual line, Q16, x 7.5^(0.5 - k)"""
    g = lattice_consts()
    return [int(round(g['STRP'] * 7.5 ** (0.5 - i / 16))) for i in range(16)]

def fixed_lattice():
    """Q8 rows per virtual line (fine, coarse) and Q8 row shear per column (fine 8 px, coarse 16 px)"""
    g = lattice_consts()
    return g['ROWF'] >> 8, g['ROWC'] >> 8, g['SHF'] >> 8, g['SHC'] >> 8

def zone_rows():
    """per zone: muH8 (256 / turn), AH (4 bits), bead threshold code T (signed 6-bit), muV, AV (4 bits), hue-noise
    select S for the line engine:  h = q (S 0), q + p (S 1), q - p (S 2);  H8 = muH8 + (h AH 2^AHSH + st KS) >> 4,
    V8 = muV + (p AV + st KV) >> 3;  a bead (q > 2 T + 1) takes H8 = HB + st KS >> 4, V8 = muV + VB + (p AV + st KV) >> 3.
    The noise p, q in -64..63 (n = t ns / 32), the stripe st in -32..31.  Returns (rows, AHSH)."""
    ns = TEX['ns']
    raw = []
    for (mu, sdh, vm, vs, rho), fr in zip(ZONEHSV, ZRED):
        S = 1 if rho > TEX['rth'] else (2 if rho < -TEX['rth'] else 0)
        aH = TEX['gH'] * sdh / (math.sqrt(2) if S else 1.0)
        aV = TEX['gV'] * vs; muV = TEX['mV'] * vm + TEX['dV']; muH = mu + TEX['dH'] / 360
        AV = min(15, int(round(aV * ns / 16)))
        f = min(0.6, fr * TEX['bsc'])
        T = 31 if f < 0.003 else min(range(-32, 32), key=lambda t: abs(QTAIL[t + 32] - f))
        mV8 = max(0, min(255, int(round(muV))))
        if T < 31: assert 0 <= mV8 + bead_consts()[1] <= 255, (mV8, T)
        raw.append((int(round(muH * 256)) % 256, aH * ns * 32, T, mV8, AV, S))
    AHSH = next(k for k in (0, 1, 2) if max(r[1] for r in raw) * 2 ** (2 - k) <= 15.49)
    rows = [(m, int(round(a * 2 ** (2 - AHSH))), T, v, av, S) for m, a, T, v, av, S in raw]
    return rows, AHSH

def bead_consts():
    """HB (bead hue, 256 / turn) and VB (bead value offset)"""
    return int(round(TEX['hb'] / 360 * 256)) % 256, int(round(TEX['vb']))

def stripe_floor():
    """SC: the stripe triangle st (-32..32) is floored at SC, so the ridges are one-sided (sc = -1: a plain triangle)"""
    return max(-32, min(31, int(round(TEX.get('sc', -1.0) * 32))))

def stripe_wander():
    """SWSH: the ridge phase (16 bits a cycle) wanders by the hue noise q << SWSH (sw = 2^SWSH / 1024 cycle per q/64)"""
    sh = int(round(math.log2(TEX['sw'] * 1024)))
    assert abs(2 ** sh / 1024 - TEX['sw']) < 1e-6, TEX['sw']
    return sh

def stripe_consts():
    """KS (hue), KV (value) per stripe unit, st in -32..31: H8 += st KS >> 4, V8 += st KV >> 3"""
    KS = int(round(TEX['ks'] * 128)); KV = int(round(TEX['kv'] / 4))
    assert abs(KS) < 64 and abs(KV) < 64, (KS, KV)
    return KS, KV

def colour_rows():
    """256 hues: (Y, Cr, Cb) of the full-saturation colour (value 1), x 64 per value unit in 10-bit codes"""
    out = []
    for i in range(256):
        h6 = i / 256 * 6; s = int(h6) % 6; f = h6 - int(h6)
        R, G, B = [(1, f, 0), (1 - f, 1, 0), (0, 1, f), (0, 1 - f, 1), (f, 0, 1), (1, 0, 1 - f)][s]
        Y = 0.299 * R + 0.587 * G + 0.114 * B
        Cr = (R - Y) / 1.402; Cb = (B - Y) / 1.772
        out.append((int(round(Y * 876 / 255 * 64)), int(round(Cr * 896 / 255 * 64)), int(round(Cb * 896 / 255 * 64))))
    return out

def slv(v, w):
    return '"' + format(v & ((1 << w) - 1), '0%db' % w) + '"'

def main():
    K = knots()
    lines = []
    w = lines.append
    w('-- riptide_pkg.vhd -- GENERATED by riptide.py; edit the generator, not this file.')
    w('--')
    w('-- C_RIP_KA*/KB*: the band field (256 x 32 polar knots H6 V4, even / odd rows).  C_RIP_ZT: the texture')
    w('-- zone table.  C_RIP_COL: hue -> YUV colourizer (U = Cr).  C_RIP_GSTRP: K6 Ridges table.')
    w('')
    w('library ieee;')
    w('use ieee.std_logic_1164.all;')
    w('')
    w('package riptide_pkg is')
    w('    type t_rip_k8 is array(0 to 4095) of std_logic_vector(7 downto 0);')
    w('    type t_rip_k2 is array(0 to 4095) of std_logic_vector(1 downto 0);')
    for par, nm in ((0, 'E'), (1, 'O')):
        a_ent = []; b_ent = []
        for rp in range(16):
            r = 2 * rp + par
            for c in range(256):
                h, v = K[r][c]
                a_ent.append((h << 2) | (v >> 2)); b_ent.append(v & 3)
        w('    constant C_RIP_KA%s : t_rip_k8 := (' % nm)
        for i in range(0, 4096, 16):
            w('        ' + ', '.join(slv(x, 8) for x in a_ent[i:i + 16]) + (',' if i + 16 < 4096 else ''))
        w('    );')
        w('    constant C_RIP_KB%s : t_rip_k2 := (' % nm)
        for i in range(0, 4096, 32):
            w('        ' + ', '.join(slv(x, 2) for x in b_ent[i:i + 32]) + (',' if i + 32 < 4096 else ''))
        w('    );')
    w('    -- zone table: muH 34:27 | AH 24:21 | bead threshold T (signed) 20:15 | muV 13:6 | AV 3:0')
    ZR, AHSH = zone_rows()
    w('    type t_rip_zt is array(0 to 31) of std_logic_vector(34 downto 0);')
    w('    constant C_RIP_ZT : t_rip_zt := (')
    ents = []
    for muH, AH, T, muV, AV, S in ZR:
        v = (muH << 27) | (AH << 21) | ((T & 63) << 15) | (muV << 6) | AV
        ents.append(slv(v, 35))
    for i in range(0, 32, 4):
        w('        ' + ', '.join(ents[i:i + 4]) + (',' if i + 4 < 32 else ''))
    w('    );')
    w('    -- hue-noise select per zone: 0 = q, 1 = q + p, 2 = q - p (the zone hue / value correlation)')
    w('    type t_rip_zs is array(0 to 31) of integer range 0 to 2;')
    w('    constant C_RIP_ZS : t_rip_zs := (%s);' % ', '.join(str(r[5]) for r in ZR))
    w('    constant C_RIP_AHSH : integer := %d;   -- hue spread scale 2^AHSH' % AHSH)
    KS, KV = stripe_consts()
    w('    constant C_RIP_KS : integer := %d;' % KS)
    w('    constant C_RIP_KV : integer := %d;' % KV)
    w('    constant C_RIP_SC : integer := %d;   -- stripe floor' % stripe_floor())
    w('    constant C_RIP_SWSH : integer := %d;   -- stripe phase wander q << SWSH' % stripe_wander())
    w('    constant C_RIP_TFL : integer := %d;   -- texture floor (RGB)' % int(round(TEX['fl'])))
    HB, VB = bead_consts()
    w('    constant C_RIP_HB : integer := %d;   -- bead hue (256 / turn)' % HB)
    w('    constant C_RIP_VB : integer := %d;   -- bead value offset' % VB)
    w('    type t_rip_col is array(0 to 255) of std_logic_vector(23 downto 0);')
    w('    constant C_RIP_COL : t_rip_col := (')
    ents = [slv((y << 16) | ((cr & 255) << 8) | (cb & 255), 24) for y, cr, cb in colour_rows()]
    for i in range(0, 256, 8):
        w('        ' + ', '.join(ents[i:i + 8]) + (',' if i + 8 < 256 else ''))
    w('    );')
    G = grain16()
    w('    type t_rip_gstrp is array(0 to 15) of integer range 0 to 32767;')
    w('    constant C_RIP_GSTRP : t_rip_gstrp := (%s);' % ', '.join(str(v) for v in G))
    rf, rc, sf, sc = fixed_lattice()
    w('    constant C_RIP_ROWF : integer := %d;   -- fine lattice rows per virtual line, Q8' % rf)
    w('    constant C_RIP_ROWC : integer := %d;   -- coarse lattice rows per virtual line, Q8' % rc)
    w('    constant C_RIP_SHF : integer := %d;    -- fine row shear per 8-px column, Q8' % sf)
    w('    constant C_RIP_SHC : integer := %d;    -- coarse row shear per 16-px column, Q8' % sc)
    w('    constant C_RIP_ZPX : integer := %d;  -- zone phase per virtual px, Q24 (5.121 columns / 1920 px)' % ZPX_K)
    w('end package riptide_pkg;')
    with open(os.path.join(HERE, 'riptide_pkg.vhd'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

if __name__ == '__main__':
    main()
