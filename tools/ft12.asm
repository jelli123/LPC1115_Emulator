
.\tools\ft12.bin:     file format binary


Disassembly of section .data:

00000000 <.data>:
       0:	2000      	movs	r0, #0
       2:	1000      	asrs	r0, r0, #32
       4:	00e5      	lsls	r5, r4, #3
       6:	0000      	movs	r0, r0
       8:	00d9      	lsls	r1, r3, #3
       a:	0000      	movs	r0, r0
       c:	1561      	asrs	r1, r4, #21
	...
      1a:	0000      	movs	r0, r0
      1c:	df19      	svc	25
      1e:	efff 0000 	vext.8	d16, d15, d0, #0
	...
      2a:	0000      	movs	r0, r0
      2c:	00dd      	lsls	r5, r3, #3
	...
      36:	0000      	movs	r0, r0
      38:	00e1      	lsls	r1, r4, #3
      3a:	0000      	movs	r0, r0
      3c:	0639      	lsls	r1, r7, #24
      3e:	0000      	movs	r0, r0
      40:	00d5      	lsls	r5, r2, #3
      42:	0000      	movs	r0, r0
      44:	00d5      	lsls	r5, r2, #3
      46:	0000      	movs	r0, r0
      48:	00d5      	lsls	r5, r2, #3
      4a:	0000      	movs	r0, r0
      4c:	00d5      	lsls	r5, r2, #3
      4e:	0000      	movs	r0, r0
      50:	00d5      	lsls	r5, r2, #3
      52:	0000      	movs	r0, r0
      54:	00d5      	lsls	r5, r2, #3
      56:	0000      	movs	r0, r0
      58:	00d5      	lsls	r5, r2, #3
      5a:	0000      	movs	r0, r0
      5c:	00d5      	lsls	r5, r2, #3
      5e:	0000      	movs	r0, r0
      60:	00d5      	lsls	r5, r2, #3
      62:	0000      	movs	r0, r0
      64:	00d5      	lsls	r5, r2, #3
      66:	0000      	movs	r0, r0
      68:	00d5      	lsls	r5, r2, #3
      6a:	0000      	movs	r0, r0
      6c:	00d5      	lsls	r5, r2, #3
      6e:	0000      	movs	r0, r0
      70:	00d5      	lsls	r5, r2, #3
      72:	0000      	movs	r0, r0
      74:	00d5      	lsls	r5, r2, #3
      76:	0000      	movs	r0, r0
      78:	00d5      	lsls	r5, r2, #3
      7a:	0000      	movs	r0, r0
      7c:	00d5      	lsls	r5, r2, #3
      7e:	0000      	movs	r0, r0
      80:	00d5      	lsls	r5, r2, #3
      82:	0000      	movs	r0, r0
      84:	14ad      	asrs	r5, r5, #18
      86:	0000      	movs	r0, r0
      88:	00d5      	lsls	r5, r2, #3
      8a:	0000      	movs	r0, r0
      8c:	00d5      	lsls	r5, r2, #3
      8e:	0000      	movs	r0, r0
      90:	00d5      	lsls	r5, r2, #3
      92:	0000      	movs	r0, r0
      94:	0629      	lsls	r1, r5, #24
	...
      9e:	0000      	movs	r0, r0
      a0:	00d5      	lsls	r5, r2, #3
      a2:	0000      	movs	r0, r0
      a4:	00d5      	lsls	r5, r2, #3
      a6:	0000      	movs	r0, r0
      a8:	00d5      	lsls	r5, r2, #3
      aa:	0000      	movs	r0, r0
      ac:	0000      	movs	r0, r0
      ae:	0000      	movs	r0, r0
      b0:	00d5      	lsls	r5, r2, #3
      b2:	0000      	movs	r0, r0
      b4:	00d5      	lsls	r5, r2, #3
      b6:	0000      	movs	r0, r0
      b8:	00d5      	lsls	r5, r2, #3
      ba:	0000      	movs	r0, r0
      bc:	00d5      	lsls	r5, r2, #3
      be:	0000      	movs	r0, r0
      c0:	31c4      	adds	r1, #196	@ 0xc4
      c2:	0000      	movs	r0, r0
      c4:	0000      	movs	r0, r0
      c6:	1000      	asrs	r0, r0, #32
      c8:	0070      	lsls	r0, r6, #1
      ca:	0000      	movs	r0, r0
      cc:	0070      	lsls	r0, r6, #1
      ce:	1000      	asrs	r0, r0, #32
      d0:	026c      	lsls	r4, r5, #9
      d2:	0000      	movs	r0, r0
      d4:	e7fe      	b.n	0xd4
      d6:	46c0      	nop			@ (mov r8, r8)
      d8:	e7fe      	b.n	0xd8
      da:	46c0      	nop			@ (mov r8, r8)
      dc:	e7fe      	b.n	0xdc
      de:	46c0      	nop			@ (mov r8, r8)
      e0:	e7fe      	b.n	0xe0
      e2:	46c0      	nop			@ (mov r8, r8)
      e4:	b5f0      	push	{r4, r5, r6, r7, lr}
      e6:	4b58      	ldr	r3, [pc, #352]	@ (0x248)
      e8:	4f58      	ldr	r7, [pc, #352]	@ (0x24c)
      ea:	b085      	sub	sp, #20
      ec:	9302      	str	r3, [sp, #8]
      ee:	429f      	cmp	r7, r3
      f0:	d300      	bcc.n	0xf4
      f2:	e0a7      	b.n	0x244
      f4:	2300      	movs	r3, #0
      f6:	469c      	mov	ip, r3
      f8:	003b      	movs	r3, r7
      fa:	689b      	ldr	r3, [r3, #8]
      fc:	683a      	ldr	r2, [r7, #0]
      fe:	6879      	ldr	r1, [r7, #4]
     100:	370c      	adds	r7, #12
     102:	9701      	str	r7, [sp, #4]
     104:	2b00      	cmp	r3, #0
     106:	d032      	beq.n	0x16e
     108:	1e5c      	subs	r4, r3, #1
     10a:	2c27      	cmp	r4, #39	@ 0x27
     10c:	d96d      	bls.n	0x1ea
     10e:	0010      	movs	r0, r2
     110:	2507      	movs	r5, #7
     112:	4308      	orrs	r0, r1
     114:	4028      	ands	r0, r5
     116:	4246      	negs	r6, r0
     118:	4146      	adcs	r6, r0
     11a:	2004      	movs	r0, #4
     11c:	4665      	mov	r5, ip
     11e:	4240      	negs	r0, r0
     120:	4298      	cmp	r0, r3
     122:	416d      	adcs	r5, r5
     124:	9600      	str	r6, [sp, #0]
     126:	9e00      	ldr	r6, [sp, #0]
     128:	4235      	tst	r5, r6
     12a:	d05e      	beq.n	0x1ea
     12c:	1d10      	adds	r0, r2, #4
     12e:	4281      	cmp	r1, r0
     130:	d05b      	beq.n	0x1ea
     132:	08a0      	lsrs	r0, r4, #2
     134:	3001      	adds	r0, #1
     136:	0845      	lsrs	r5, r0, #1
     138:	00ed      	lsls	r5, r5, #3
     13a:	18ad      	adds	r5, r5, r2
     13c:	1a8e      	subs	r6, r1, r2
     13e:	9103      	str	r1, [sp, #12]
     140:	0013      	movs	r3, r2
     142:	0029      	movs	r1, r5
     144:	9200      	str	r2, [sp, #0]
     146:	18f2      	adds	r2, r6, r3
     148:	681c      	ldr	r4, [r3, #0]
     14a:	685d      	ldr	r5, [r3, #4]
     14c:	3308      	adds	r3, #8
     14e:	6014      	str	r4, [r2, #0]
     150:	6055      	str	r5, [r2, #4]
     152:	428b      	cmp	r3, r1
     154:	d1f7      	bne.n	0x146
     156:	0003      	movs	r3, r0
     158:	2401      	movs	r4, #1
     15a:	9a00      	ldr	r2, [sp, #0]
     15c:	9903      	ldr	r1, [sp, #12]
     15e:	43a3      	bics	r3, r4
     160:	009b      	lsls	r3, r3, #2
     162:	18d2      	adds	r2, r2, r3
     164:	18c9      	adds	r1, r1, r3
     166:	4204      	tst	r4, r0
     168:	d001      	beq.n	0x16e
     16a:	6813      	ldr	r3, [r2, #0]
     16c:	600b      	str	r3, [r1, #0]
     16e:	9b02      	ldr	r3, [sp, #8]
     170:	42bb      	cmp	r3, r7
     172:	d8c1      	bhi.n	0xf8
     174:	4b36      	ldr	r3, [pc, #216]	@ (0x250)
     176:	9a01      	ldr	r2, [sp, #4]
     178:	429a      	cmp	r2, r3
     17a:	d21c      	bcs.n	0x1b6
     17c:	3b01      	subs	r3, #1
     17e:	0014      	movs	r4, r2
     180:	1a9b      	subs	r3, r3, r2
     182:	2207      	movs	r2, #7
     184:	4393      	bics	r3, r2
     186:	0022      	movs	r2, r4
     188:	3208      	adds	r2, #8
     18a:	189b      	adds	r3, r3, r2
     18c:	001f      	movs	r7, r3
     18e:	2301      	movs	r3, #1
     190:	2100      	movs	r1, #0
     192:	469c      	mov	ip, r3
     194:	6862      	ldr	r2, [r4, #4]
     196:	6820      	ldr	r0, [r4, #0]
     198:	2a00      	cmp	r2, #0
     19a:	d009      	beq.n	0x1b0
     19c:	1e55      	subs	r5, r2, #1
     19e:	2300      	movs	r3, #0
     1a0:	2d67      	cmp	r5, #103	@ 0x67
     1a2:	d901      	bls.n	0x1a8
     1a4:	1d16      	adds	r6, r2, #4
     1a6:	d927      	bls.n	0x1f8
     1a8:	50c1      	str	r1, [r0, r3]
     1aa:	3304      	adds	r3, #4
     1ac:	429a      	cmp	r2, r3
     1ae:	d8fb      	bhi.n	0x1a8
     1b0:	3408      	adds	r4, #8
     1b2:	42a7      	cmp	r7, r4
     1b4:	d1ee      	bne.n	0x194
     1b6:	f002 fc9b 	bl	0x2af0
     1ba:	f002 fe4b 	bl	0x2e54
     1be:	21fa      	movs	r1, #250	@ 0xfa
     1c0:	4b24      	ldr	r3, [pc, #144]	@ (0x254)
     1c2:	0089      	lsls	r1, r1, #2
     1c4:	6818      	ldr	r0, [r3, #0]
     1c6:	f002 fcd7 	bl	0x2b78
     1ca:	f000 fb27 	bl	0x81c
     1ce:	f001 fa83 	bl	0x16d8
     1d2:	0004      	movs	r4, r0
     1d4:	6823      	ldr	r3, [r4, #0]
     1d6:	0020      	movs	r0, r4
     1d8:	689b      	ldr	r3, [r3, #8]
     1da:	4798      	blx	r3
     1dc:	6823      	ldr	r3, [r4, #0]
     1de:	0020      	movs	r0, r4
     1e0:	6adb      	ldr	r3, [r3, #44]	@ 0x2c
     1e2:	4798      	blx	r3
     1e4:	f001 fea4 	bl	0x1f30
     1e8:	e7f4      	b.n	0x1d4
     1ea:	2000      	movs	r0, #0
     1ec:	5814      	ldr	r4, [r2, r0]
     1ee:	500c      	str	r4, [r1, r0]
     1f0:	3004      	adds	r0, #4
     1f2:	4283      	cmp	r3, r0
     1f4:	d8fa      	bhi.n	0x1ec
     1f6:	e7ba      	b.n	0x16e
     1f8:	08ad      	lsrs	r5, r5, #2
     1fa:	1c6b      	adds	r3, r5, #1
     1fc:	9300      	str	r3, [sp, #0]
     1fe:	4666      	mov	r6, ip
     200:	4663      	mov	r3, ip
     202:	0885      	lsrs	r5, r0, #2
     204:	426d      	negs	r5, r5
     206:	0002      	movs	r2, r0
     208:	402b      	ands	r3, r5
     20a:	422e      	tst	r6, r5
     20c:	d000      	beq.n	0x210
     20e:	c202      	stmia	r2!, {r1}
     210:	9d00      	ldr	r5, [sp, #0]
     212:	1aed      	subs	r5, r5, r3
     214:	009b      	lsls	r3, r3, #2
     216:	181b      	adds	r3, r3, r0
     218:	0868      	lsrs	r0, r5, #1
     21a:	00c0      	lsls	r0, r0, #3
     21c:	18c0      	adds	r0, r0, r3
     21e:	9500      	str	r5, [sp, #0]
     220:	2500      	movs	r5, #0
     222:	2600      	movs	r6, #0
     224:	c360      	stmia	r3!, {r5, r6}
     226:	4298      	cmp	r0, r3
     228:	d1fa      	bne.n	0x220
     22a:	9d00      	ldr	r5, [sp, #0]
     22c:	4660      	mov	r0, ip
     22e:	002b      	movs	r3, r5
     230:	4383      	bics	r3, r0
     232:	009b      	lsls	r3, r3, #2
     234:	18d2      	adds	r2, r2, r3
     236:	4228      	tst	r0, r5
     238:	d0ba      	beq.n	0x1b0
     23a:	3408      	adds	r4, #8
     23c:	6011      	str	r1, [r2, #0]
     23e:	42a7      	cmp	r7, r4
     240:	d1a8      	bne.n	0x194
     242:	e7b8      	b.n	0x1b6
     244:	9701      	str	r7, [sp, #4]
     246:	e795      	b.n	0x174
     248:	00cc      	lsls	r4, r1, #3
     24a:	0000      	movs	r0, r0
     24c:	00c0      	lsls	r0, r0, #3
     24e:	0000      	movs	r0, r0
     250:	00d4      	lsls	r4, r2, #3
     252:	0000      	movs	r0, r0
     254:	001c      	movs	r4, r3
     256:	1000      	asrs	r0, r0, #32
     258:	0003      	movs	r3, r0
     25a:	33ff      	adds	r3, #255	@ 0xff
     25c:	7e5b      	ldrb	r3, [r3, #25]
     25e:	2b00      	cmp	r3, #0
     260:	d003      	beq.n	0x26a
     262:	6902      	ldr	r2, [r0, #16]
     264:	6943      	ldr	r3, [r0, #20]
     266:	429a      	cmp	r2, r3
     268:	d1fb      	bne.n	0x262
     26a:	4770      	bx	lr
     26c:	0002      	movs	r2, r0
     26e:	32ff      	adds	r2, #255	@ 0xff
     270:	7e52      	ldrb	r2, [r2, #25]
     272:	0003      	movs	r3, r0
     274:	1e10      	subs	r0, r2, #0
     276:	d00d      	beq.n	0x294
     278:	6918      	ldr	r0, [r3, #16]
     27a:	695a      	ldr	r2, [r3, #20]
     27c:	4290      	cmp	r0, r2
     27e:	d10a      	bne.n	0x296
     280:	4a0c      	ldr	r2, [pc, #48]	@ (0x2b4)
     282:	6950      	ldr	r0, [r2, #20]
     284:	0680      	lsls	r0, r0, #26
     286:	d506      	bpl.n	0x296
     288:	6011      	str	r1, [r2, #0]
     28a:	2302      	movs	r3, #2
     28c:	2001      	movs	r0, #1
     28e:	6851      	ldr	r1, [r2, #4]
     290:	430b      	orrs	r3, r1
     292:	6053      	str	r3, [r2, #4]
     294:	4770      	bx	lr
     296:	207f      	movs	r0, #127	@ 0x7f
     298:	695a      	ldr	r2, [r3, #20]
     29a:	3201      	adds	r2, #1
     29c:	4002      	ands	r2, r0
     29e:	6918      	ldr	r0, [r3, #16]
     2a0:	4282      	cmp	r2, r0
     2a2:	d0fc      	beq.n	0x29e
     2a4:	6958      	ldr	r0, [r3, #20]
     2a6:	1818      	adds	r0, r3, r0
     2a8:	3098      	adds	r0, #152	@ 0x98
     2aa:	7001      	strb	r1, [r0, #0]
     2ac:	615a      	str	r2, [r3, #20]
     2ae:	4a01      	ldr	r2, [pc, #4]	@ (0x2b4)
     2b0:	e7eb      	b.n	0x28a
     2b2:	46c0      	nop			@ (mov r8, r8)
     2b4:	8000      	strh	r0, [r0, #0]
     2b6:	4000      	ands	r0, r0
     2b8:	228c      	movs	r2, #140	@ 0x8c
     2ba:	4b18      	ldr	r3, [pc, #96]	@ (0x31c)
     2bc:	0052      	lsls	r2, r2, #1
     2be:	5c9a      	ldrb	r2, [r3, r2]
     2c0:	2a00      	cmp	r2, #0
     2c2:	d019      	beq.n	0x2f8
     2c4:	6919      	ldr	r1, [r3, #16]
     2c6:	695a      	ldr	r2, [r3, #20]
     2c8:	4291      	cmp	r1, r2
     2ca:	d01c      	beq.n	0x306
     2cc:	217f      	movs	r1, #127	@ 0x7f
     2ce:	695a      	ldr	r2, [r3, #20]
     2d0:	3201      	adds	r2, #1
     2d2:	4011      	ands	r1, r2
     2d4:	691a      	ldr	r2, [r3, #16]
     2d6:	4291      	cmp	r1, r2
     2d8:	d0fc      	beq.n	0x2d4
     2da:	20e5      	movs	r0, #229	@ 0xe5
     2dc:	695a      	ldr	r2, [r3, #20]
     2de:	6159      	str	r1, [r3, #20]
     2e0:	189a      	adds	r2, r3, r2
     2e2:	490f      	ldr	r1, [pc, #60]	@ (0x320)
     2e4:	3298      	adds	r2, #152	@ 0x98
     2e6:	7010      	strb	r0, [r2, #0]
     2e8:	684a      	ldr	r2, [r1, #4]
     2ea:	38e3      	subs	r0, #227	@ 0xe3
     2ec:	4302      	orrs	r2, r0
     2ee:	604a      	str	r2, [r1, #4]
     2f0:	6919      	ldr	r1, [r3, #16]
     2f2:	695a      	ldr	r2, [r3, #20]
     2f4:	4291      	cmp	r1, r2
     2f6:	d1fb      	bne.n	0x2f0
     2f8:	22a0      	movs	r2, #160	@ 0xa0
     2fa:	2380      	movs	r3, #128	@ 0x80
     2fc:	2140      	movs	r1, #64	@ 0x40
     2fe:	05d2      	lsls	r2, r2, #23
     300:	005b      	lsls	r3, r3, #1
     302:	50d1      	str	r1, [r2, r3]
     304:	4770      	bx	lr
     306:	4906      	ldr	r1, [pc, #24]	@ (0x320)
     308:	694a      	ldr	r2, [r1, #20]
     30a:	0692      	lsls	r2, r2, #26
     30c:	d5de      	bpl.n	0x2cc
     30e:	22e5      	movs	r2, #229	@ 0xe5
     310:	2002      	movs	r0, #2
     312:	600a      	str	r2, [r1, #0]
     314:	684a      	ldr	r2, [r1, #4]
     316:	4302      	orrs	r2, r0
     318:	604a      	str	r2, [r1, #4]
     31a:	e7e9      	b.n	0x2f0
     31c:	0194      	lsls	r4, r2, #6
     31e:	1000      	asrs	r0, r0, #32
     320:	8000      	strh	r0, [r0, #0]
     322:	4000      	ands	r0, r0
     324:	2017      	movs	r0, #23
     326:	4770      	bx	lr
     328:	4770      	bx	lr
     32a:	46c0      	nop			@ (mov r8, r8)
     32c:	6900      	ldr	r0, [r0, #16]
     32e:	3062      	adds	r0, #98	@ 0x62
     330:	4770      	bx	lr
     332:	46c0      	nop			@ (mov r8, r8)
     334:	6900      	ldr	r0, [r0, #16]
     336:	3063      	adds	r0, #99	@ 0x63
     338:	4770      	bx	lr
     33a:	46c0      	nop			@ (mov r8, r8)
     33c:	2060      	movs	r0, #96	@ 0x60
     33e:	4770      	bx	lr
     340:	2061      	movs	r0, #97	@ 0x61
     342:	4770      	bx	lr
     344:	8281      	strh	r1, [r0, #20]
     346:	4770      	bx	lr
     348:	7900      	ldrb	r0, [r0, #4]
     34a:	4770      	bx	lr
     34c:	2000      	movs	r0, #0
     34e:	4770      	bx	lr
     350:	2001      	movs	r0, #1
     352:	4770      	bx	lr
     354:	2001      	movs	r0, #1
     356:	4770      	bx	lr
     358:	2000      	movs	r0, #0
     35a:	4770      	bx	lr
     35c:	23e0      	movs	r3, #224	@ 0xe0
     35e:	009b      	lsls	r3, r3, #2
     360:	4299      	cmp	r1, r3
     362:	d110      	bne.n	0x386
     364:	0003      	movs	r3, r0
     366:	2201      	movs	r2, #1
     368:	3348      	adds	r3, #72	@ 0x48
     36a:	701a      	strb	r2, [r3, #0]
     36c:	7943      	ldrb	r3, [r0, #5]
     36e:	1e59      	subs	r1, r3, #1
     370:	418b      	sbcs	r3, r1
     372:	0001      	movs	r1, r0
     374:	3149      	adds	r1, #73	@ 0x49
     376:	700b      	strb	r3, [r1, #0]
     378:	4b05      	ldr	r3, [pc, #20]	@ (0x390)
     37a:	681b      	ldr	r3, [r3, #0]
     37c:	0019      	movs	r1, r3
     37e:	31fa      	adds	r1, #250	@ 0xfa
     380:	33fa      	adds	r3, #250	@ 0xfa
     382:	d002      	beq.n	0x38a
     384:	64c1      	str	r1, [r0, #76]	@ 0x4c
     386:	2000      	movs	r0, #0
     388:	4770      	bx	lr
     38a:	64c2      	str	r2, [r0, #76]	@ 0x4c
     38c:	e7fb      	b.n	0x386
     38e:	46c0      	nop			@ (mov r8, r8)
     390:	0070      	lsls	r0, r6, #1
     392:	1000      	asrs	r0, r0, #32
     394:	2200      	movs	r2, #0
     396:	6a83      	ldr	r3, [r0, #40]	@ 0x28
     398:	605a      	str	r2, [r3, #4]
     39a:	4770      	bx	lr
     39c:	6ac3      	ldr	r3, [r0, #44]	@ 0x2c
     39e:	2b00      	cmp	r3, #0
     3a0:	d109      	bne.n	0x3b6
     3a2:	0002      	movs	r2, r0
     3a4:	325f      	adds	r2, #95	@ 0x5f
     3a6:	7813      	ldrb	r3, [r2, #0]
     3a8:	305a      	adds	r0, #90	@ 0x5a
     3aa:	3301      	adds	r3, #1
     3ac:	7013      	strb	r3, [r2, #0]
     3ae:	2340      	movs	r3, #64	@ 0x40
     3b0:	8802      	ldrh	r2, [r0, #0]
     3b2:	4313      	orrs	r3, r2
     3b4:	8003      	strh	r3, [r0, #0]
     3b6:	4770      	bx	lr
     3b8:	2300      	movs	r3, #0
     3ba:	1d42      	adds	r2, r0, #5
     3bc:	7143      	strb	r3, [r0, #5]
     3be:	77d3      	strb	r3, [r2, #31]
     3c0:	1d82      	adds	r2, r0, #6
     3c2:	77d3      	strb	r3, [r2, #31]
     3c4:	1dc2      	adds	r2, r0, #7
     3c6:	77d3      	strb	r3, [r2, #31]
     3c8:	2201      	movs	r2, #1
     3ca:	4252      	negs	r2, r2
     3cc:	8102      	strh	r2, [r0, #8]
     3ce:	3202      	adds	r2, #2
     3d0:	7102      	strb	r2, [r0, #4]
     3d2:	4a03      	ldr	r2, [pc, #12]	@ (0x3e0)
     3d4:	80c3      	strh	r3, [r0, #6]
     3d6:	60c3      	str	r3, [r0, #12]
     3d8:	8093      	strh	r3, [r2, #4]
     3da:	80d3      	strh	r3, [r2, #6]
     3dc:	4770      	bx	lr
     3de:	46c0      	nop			@ (mov r8, r8)
     3e0:	0070      	lsls	r0, r6, #1
     3e2:	1000      	asrs	r0, r0, #32
     3e4:	2202      	movs	r2, #2
     3e6:	1d43      	adds	r3, r0, #5
     3e8:	b510      	push	{r4, lr}
     3ea:	77da      	strb	r2, [r3, #31]
     3ec:	230f      	movs	r3, #15
     3ee:	6981      	ldr	r1, [r0, #24]
     3f0:	794a      	ldrb	r2, [r1, #5]
     3f2:	401a      	ands	r2, r3
     3f4:	6803      	ldr	r3, [r0, #0]
     3f6:	3207      	adds	r2, #7
     3f8:	6a5b      	ldr	r3, [r3, #36]	@ 0x24
     3fa:	4798      	blx	r3
     3fc:	bd10      	pop	{r4, pc}
     3fe:	b570      	push	{r4, r5, r6, lr}
     400:	0005      	movs	r5, r0
     402:	6bc3      	ldr	r3, [r0, #60]	@ 0x3c
     404:	0004      	movs	r4, r0
     406:	355a      	adds	r5, #90	@ 0x5a
     408:	2b00      	cmp	r3, #0
     40a:	d013      	beq.n	0x434
     40c:	2200      	movs	r2, #0
     40e:	63c2      	str	r2, [r0, #60]	@ 0x3c
     410:	6880      	ldr	r0, [r0, #8]
     412:	882b      	ldrh	r3, [r5, #0]
     414:	1d41      	adds	r1, r0, #5
     416:	77ca      	strb	r2, [r1, #31]
     418:	6982      	ldr	r2, [r0, #24]
     41a:	7992      	ldrb	r2, [r2, #6]
     41c:	2a7f      	cmp	r2, #127	@ 0x7f
     41e:	d909      	bls.n	0x434
     420:	061b      	lsls	r3, r3, #24
     422:	d50e      	bpl.n	0x442
     424:	230b      	movs	r3, #11
     426:	56c3      	ldrsb	r3, [r0, r3]
     428:	2b02      	cmp	r3, #2
     42a:	dc0a      	bgt.n	0x442
     42c:	3301      	adds	r3, #1
     42e:	72c3      	strb	r3, [r0, #11]
     430:	f7ff ffd8 	bl	0x3e4
     434:	2300      	movs	r3, #0
     436:	61e3      	str	r3, [r4, #28]
     438:	6263      	str	r3, [r4, #36]	@ 0x24
     43a:	63a3      	str	r3, [r4, #56]	@ 0x38
     43c:	802b      	strh	r3, [r5, #0]
     43e:	65e3      	str	r3, [r4, #92]	@ 0x5c
     440:	bd70      	pop	{r4, r5, r6, pc}
     442:	2300      	movs	r3, #0
     444:	72c3      	strb	r3, [r0, #11]
     446:	e7f5      	b.n	0x434
     448:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
     44a:	000c      	movs	r4, r1
     44c:	1d41      	adds	r1, r0, #5
     44e:	7fcd      	ldrb	r5, [r1, #31]
     450:	2d00      	cmp	r5, #0
     452:	d1fc      	bne.n	0x44e
     454:	2640      	movs	r6, #64	@ 0x40
     456:	3501      	adds	r5, #1
     458:	77cd      	strb	r5, [r1, #31]
     45a:	0a17      	lsrs	r7, r2, #8
     45c:	6985      	ldr	r5, [r0, #24]
     45e:	b2d2      	uxtb	r2, r2
     460:	4234      	tst	r4, r6
     462:	d003      	beq.n	0x46c
     464:	0099      	lsls	r1, r3, #2
     466:	4321      	orrs	r1, r4
     468:	430e      	orrs	r6, r1
     46a:	b2f4      	uxtb	r4, r6
     46c:	23b0      	movs	r3, #176	@ 0xb0
     46e:	702b      	strb	r3, [r5, #0]
     470:	3b50      	subs	r3, #80	@ 0x50
     472:	70ef      	strb	r7, [r5, #3]
     474:	712a      	strb	r2, [r5, #4]
     476:	716b      	strb	r3, [r5, #5]
     478:	71ac      	strb	r4, [r5, #6]
     47a:	f7ff ffb3 	bl	0x3e4
     47e:	bdf8      	pop	{r3, r4, r5, r6, r7, pc}
     480:	4a0a      	ldr	r2, [pc, #40]	@ (0x4ac)
     482:	b510      	push	{r4, lr}
     484:	8893      	ldrh	r3, [r2, #4]
     486:	0004      	movs	r4, r0
     488:	3301      	adds	r3, #1
     48a:	8093      	strh	r3, [r2, #4]
     48c:	7943      	ldrb	r3, [r0, #5]
     48e:	2b00      	cmp	r3, #0
     490:	d001      	beq.n	0x496
     492:	2300      	movs	r3, #0
     494:	7143      	strb	r3, [r0, #5]
     496:	2181      	movs	r1, #129	@ 0x81
     498:	0020      	movs	r0, r4
     49a:	88e2      	ldrh	r2, [r4, #6]
     49c:	2300      	movs	r3, #0
     49e:	f7ff ffd3 	bl	0x448
     4a2:	6823      	ldr	r3, [r4, #0]
     4a4:	0020      	movs	r0, r4
     4a6:	6a9b      	ldr	r3, [r3, #40]	@ 0x28
     4a8:	4798      	blx	r3
     4aa:	bd10      	pop	{r4, pc}
     4ac:	0070      	lsls	r0, r6, #1
     4ae:	1000      	asrs	r0, r0, #32
     4b0:	2300      	movs	r3, #0
     4b2:	1d82      	adds	r2, r0, #6
     4b4:	3007      	adds	r0, #7
     4b6:	77d3      	strb	r3, [r2, #31]
     4b8:	77c3      	strb	r3, [r0, #31]
     4ba:	4770      	bx	lr
     4bc:	6842      	ldr	r2, [r0, #4]
     4be:	0003      	movs	r3, r0
     4c0:	2000      	movs	r0, #0
     4c2:	4291      	cmp	r1, r2
     4c4:	d303      	bcc.n	0x4ce
     4c6:	689b      	ldr	r3, [r3, #8]
     4c8:	428b      	cmp	r3, r1
     4ca:	4140      	adcs	r0, r0
     4cc:	b2c0      	uxtb	r0, r0
     4ce:	4770      	bx	lr
     4d0:	0003      	movs	r3, r0
     4d2:	b510      	push	{r4, lr}
     4d4:	2000      	movs	r0, #0
     4d6:	4291      	cmp	r1, r2
     4d8:	d806      	bhi.n	0x4e8
     4da:	685c      	ldr	r4, [r3, #4]
     4dc:	42a1      	cmp	r1, r4
     4de:	d303      	bcc.n	0x4e8
     4e0:	689b      	ldr	r3, [r3, #8]
     4e2:	4293      	cmp	r3, r2
     4e4:	4140      	adcs	r0, r0
     4e6:	b2c0      	uxtb	r0, r0
     4e8:	bd10      	pop	{r4, pc}
     4ea:	6843      	ldr	r3, [r0, #4]
     4ec:	6900      	ldr	r0, [r0, #16]
     4ee:	1ac9      	subs	r1, r1, r3
     4f0:	1840      	adds	r0, r0, r1
     4f2:	4770      	bx	lr
     4f4:	6842      	ldr	r2, [r0, #4]
     4f6:	0003      	movs	r3, r0
     4f8:	1a89      	subs	r1, r1, r2
     4fa:	68c2      	ldr	r2, [r0, #12]
     4fc:	2000      	movs	r0, #0
     4fe:	3a01      	subs	r2, #1
     500:	428a      	cmp	r2, r1
     502:	d301      	bcc.n	0x508
     504:	691b      	ldr	r3, [r3, #16]
     506:	5c58      	ldrb	r0, [r3, r1]
     508:	4770      	bx	lr
     50a:	6842      	ldr	r2, [r0, #4]
     50c:	b510      	push	{r4, lr}
     50e:	1a89      	subs	r1, r1, r2
     510:	68c2      	ldr	r2, [r0, #12]
     512:	1c4c      	adds	r4, r1, #1
     514:	3a01      	subs	r2, #1
     516:	0003      	movs	r3, r0
     518:	2000      	movs	r0, #0
     51a:	4294      	cmp	r4, r2
     51c:	d804      	bhi.n	0x528
     51e:	691a      	ldr	r2, [r3, #16]
     520:	5c53      	ldrb	r3, [r2, r1]
     522:	5d10      	ldrb	r0, [r2, r4]
     524:	021b      	lsls	r3, r3, #8
     526:	4318      	orrs	r0, r3
     528:	bd10      	pop	{r4, pc}
     52a:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
     52c:	0005      	movs	r5, r0
     52e:	000c      	movs	r4, r1
     530:	2600      	movs	r6, #0
     532:	188f      	adds	r7, r1, r2
     534:	42bc      	cmp	r4, r7
     536:	d101      	bne.n	0x53c
     538:	0030      	movs	r0, r6
     53a:	bdf8      	pop	{r3, r4, r5, r6, r7, pc}
     53c:	682b      	ldr	r3, [r5, #0]
     53e:	7821      	ldrb	r1, [r4, #0]
     540:	0028      	movs	r0, r5
     542:	685b      	ldr	r3, [r3, #4]
     544:	4798      	blx	r3
     546:	3401      	adds	r4, #1
     548:	1836      	adds	r6, r6, r0
     54a:	e7f3      	b.n	0x534
     54c:	4b19      	ldr	r3, [pc, #100]	@ (0x5b4)
     54e:	b530      	push	{r4, r5, lr}
     550:	695a      	ldr	r2, [r3, #20]
     552:	0692      	lsls	r2, r2, #26
     554:	d507      	bpl.n	0x566
     556:	6901      	ldr	r1, [r0, #16]
     558:	6942      	ldr	r2, [r0, #20]
     55a:	4291      	cmp	r1, r2
     55c:	d109      	bne.n	0x572
     55e:	2102      	movs	r1, #2
     560:	685a      	ldr	r2, [r3, #4]
     562:	438a      	bics	r2, r1
     564:	605a      	str	r2, [r3, #4]
     566:	2401      	movs	r4, #1
     568:	217f      	movs	r1, #127	@ 0x7f
     56a:	695a      	ldr	r2, [r3, #20]
     56c:	4222      	tst	r2, r4
     56e:	d10d      	bne.n	0x58c
     570:	bd30      	pop	{r4, r5, pc}
     572:	217f      	movs	r1, #127	@ 0x7f
     574:	6902      	ldr	r2, [r0, #16]
     576:	1882      	adds	r2, r0, r2
     578:	3298      	adds	r2, #152	@ 0x98
     57a:	7812      	ldrb	r2, [r2, #0]
     57c:	601a      	str	r2, [r3, #0]
     57e:	6902      	ldr	r2, [r0, #16]
     580:	3201      	adds	r2, #1
     582:	6102      	str	r2, [r0, #16]
     584:	6902      	ldr	r2, [r0, #16]
     586:	400a      	ands	r2, r1
     588:	6102      	str	r2, [r0, #16]
     58a:	e7ec      	b.n	0x566
     58c:	68c2      	ldr	r2, [r0, #12]
     58e:	6885      	ldr	r5, [r0, #8]
     590:	3201      	adds	r2, #1
     592:	400a      	ands	r2, r1
     594:	42aa      	cmp	r2, r5
     596:	d00a      	beq.n	0x5ae
     598:	681d      	ldr	r5, [r3, #0]
     59a:	68c2      	ldr	r2, [r0, #12]
     59c:	1882      	adds	r2, r0, r2
     59e:	7615      	strb	r5, [r2, #24]
     5a0:	68c2      	ldr	r2, [r0, #12]
     5a2:	3201      	adds	r2, #1
     5a4:	60c2      	str	r2, [r0, #12]
     5a6:	68c2      	ldr	r2, [r0, #12]
     5a8:	400a      	ands	r2, r1
     5aa:	60c2      	str	r2, [r0, #12]
     5ac:	e7dd      	b.n	0x56a
     5ae:	681a      	ldr	r2, [r3, #0]
     5b0:	e7db      	b.n	0x56a
     5b2:	46c0      	nop			@ (mov r8, r8)
     5b4:	8000      	strh	r0, [r0, #0]
     5b6:	4000      	ands	r0, r0
     5b8:	0003      	movs	r3, r0
     5ba:	b570      	push	{r4, r5, r6, lr}
     5bc:	33ff      	adds	r3, #255	@ 0xff
     5be:	7e5b      	ldrb	r3, [r3, #25]
     5c0:	2b00      	cmp	r3, #0
     5c2:	d028      	beq.n	0x616
     5c4:	257f      	movs	r5, #127	@ 0x7f
     5c6:	68c3      	ldr	r3, [r0, #12]
     5c8:	6881      	ldr	r1, [r0, #8]
     5ca:	68c4      	ldr	r4, [r0, #12]
     5cc:	6882      	ldr	r2, [r0, #8]
     5ce:	3301      	adds	r3, #1
     5d0:	402b      	ands	r3, r5
     5d2:	4294      	cmp	r4, r2
     5d4:	d01c      	beq.n	0x610
     5d6:	6882      	ldr	r2, [r0, #8]
     5d8:	1882      	adds	r2, r0, r2
     5da:	7e14      	ldrb	r4, [r2, #24]
     5dc:	6882      	ldr	r2, [r0, #8]
     5de:	3201      	adds	r2, #1
     5e0:	6082      	str	r2, [r0, #8]
     5e2:	6882      	ldr	r2, [r0, #8]
     5e4:	402a      	ands	r2, r5
     5e6:	6082      	str	r2, [r0, #8]
     5e8:	428b      	cmp	r3, r1
     5ea:	d10f      	bne.n	0x60c
     5ec:	4b0b      	ldr	r3, [pc, #44]	@ (0x61c)
     5ee:	695b      	ldr	r3, [r3, #20]
     5f0:	07db      	lsls	r3, r3, #31
     5f2:	d50b      	bpl.n	0x60c
     5f4:	f3bf 8f4f 	dsb	sy
     5f8:	f3bf 8f6f 	isb	sy
     5fc:	2580      	movs	r5, #128	@ 0x80
     5fe:	4b08      	ldr	r3, [pc, #32]	@ (0x620)
     600:	03ad      	lsls	r5, r5, #14
     602:	67dd      	str	r5, [r3, #124]	@ 0x7c
     604:	f7ff ffa2 	bl	0x54c
     608:	4b06      	ldr	r3, [pc, #24]	@ (0x624)
     60a:	601d      	str	r5, [r3, #0]
     60c:	0020      	movs	r0, r4
     60e:	bd70      	pop	{r4, r5, r6, pc}
     610:	2401      	movs	r4, #1
     612:	4264      	negs	r4, r4
     614:	e7e8      	b.n	0x5e8
     616:	2401      	movs	r4, #1
     618:	4264      	negs	r4, r4
     61a:	e7f7      	b.n	0x60c
     61c:	8000      	strh	r0, [r0, #0]
     61e:	4000      	ands	r0, r0
     620:	e104      	b.n	0x82c
     622:	e000      	b.n	0x626
     624:	e100      	b.n	0x828
     626:	e000      	b.n	0x62a
     628:	b510      	push	{r4, lr}
     62a:	4802      	ldr	r0, [pc, #8]	@ (0x634)
     62c:	f7ff ff8e 	bl	0x54c
     630:	bd10      	pop	{r4, pc}
     632:	46c0      	nop			@ (mov r8, r8)
     634:	0194      	lsls	r4, r2, #6
     636:	1000      	asrs	r0, r0, #32
     638:	4a02      	ldr	r2, [pc, #8]	@ (0x644)
     63a:	6813      	ldr	r3, [r2, #0]
     63c:	3301      	adds	r3, #1
     63e:	6013      	str	r3, [r2, #0]
     640:	4770      	bx	lr
     642:	46c0      	nop			@ (mov r8, r8)
     644:	0070      	lsls	r0, r6, #1
     646:	1000      	asrs	r0, r0, #32
     648:	68c2      	ldr	r2, [r0, #12]
     64a:	6883      	ldr	r3, [r0, #8]
     64c:	429a      	cmp	r2, r3
     64e:	d003      	beq.n	0x658
     650:	6883      	ldr	r3, [r0, #8]
     652:	18c0      	adds	r0, r0, r3
     654:	7e00      	ldrb	r0, [r0, #24]
     656:	4770      	bx	lr
     658:	2001      	movs	r0, #1
     65a:	4240      	negs	r0, r0
     65c:	e7fb      	b.n	0x656
     65e:	68c3      	ldr	r3, [r0, #12]
     660:	6880      	ldr	r0, [r0, #8]
     662:	1a18      	subs	r0, r3, r0
     664:	d500      	bpl.n	0x668
     666:	3080      	adds	r0, #128	@ 0x80
     668:	4770      	bx	lr
     66a:	b510      	push	{r4, lr}
     66c:	4c0a      	ldr	r4, [pc, #40]	@ (0x698)
     66e:	0020      	movs	r0, r4
     670:	f7ff fdf2 	bl	0x258
     674:	f3bf 8f4f 	dsb	sy
     678:	f3bf 8f6f 	isb	sy
     67c:	2280      	movs	r2, #128	@ 0x80
     67e:	4b07      	ldr	r3, [pc, #28]	@ (0x69c)
     680:	0392      	lsls	r2, r2, #14
     682:	67da      	str	r2, [r3, #124]	@ 0x7c
     684:	4a06      	ldr	r2, [pc, #24]	@ (0x6a0)
     686:	4907      	ldr	r1, [pc, #28]	@ (0x6a4)
     688:	6fd3      	ldr	r3, [r2, #124]	@ 0x7c
     68a:	34ff      	adds	r4, #255	@ 0xff
     68c:	400b      	ands	r3, r1
     68e:	67d3      	str	r3, [r2, #124]	@ 0x7c
     690:	2300      	movs	r3, #0
     692:	7663      	strb	r3, [r4, #25]
     694:	bd10      	pop	{r4, pc}
     696:	46c0      	nop			@ (mov r8, r8)
     698:	0194      	lsls	r4, r2, #6
     69a:	1000      	asrs	r0, r0, #32
     69c:	e104      	b.n	0x8a8
     69e:	e000      	b.n	0x6a2
     6a0:	8004      	strh	r4, [r0, #0]
     6a2:	4004      	ands	r4, r0
     6a4:	efff ffff 			@ <UNDEFINED> instruction: 0xefffffff
     6a8:	2303      	movs	r3, #3
     6aa:	4359      	muls	r1, r3
     6ac:	b510      	push	{r4, lr}
     6ae:	b289      	uxth	r1, r1
     6b0:	2406      	movs	r4, #6
     6b2:	2a80      	cmp	r2, #128	@ 0x80
     6b4:	d100      	bne.n	0x6b8
     6b6:	3c04      	subs	r4, #4
     6b8:	2207      	movs	r2, #7
     6ba:	408a      	lsls	r2, r1
     6bc:	408c      	lsls	r4, r1
     6be:	6a83      	ldr	r3, [r0, #40]	@ 0x28
     6c0:	4393      	bics	r3, r2
     6c2:	4323      	orrs	r3, r4
     6c4:	6283      	str	r3, [r0, #40]	@ 0x28
     6c6:	bd10      	pop	{r4, pc}
     6c8:	b530      	push	{r4, r5, lr}
     6ca:	2403      	movs	r4, #3
     6cc:	2507      	movs	r5, #7
     6ce:	434c      	muls	r4, r1
     6d0:	40a5      	lsls	r5, r4
     6d2:	40a2      	lsls	r2, r4
     6d4:	6943      	ldr	r3, [r0, #20]
     6d6:	0049      	lsls	r1, r1, #1
     6d8:	43ab      	bics	r3, r5
     6da:	4313      	orrs	r3, r2
     6dc:	2230      	movs	r2, #48	@ 0x30
     6de:	408a      	lsls	r2, r1
     6e0:	6143      	str	r3, [r0, #20]
     6e2:	6bc3      	ldr	r3, [r0, #60]	@ 0x3c
     6e4:	4393      	bics	r3, r2
     6e6:	63c3      	str	r3, [r0, #60]	@ 0x3c
     6e8:	bd30      	pop	{r4, r5, pc}
     6ea:	b570      	push	{r4, r5, r6, lr}
     6ec:	6a83      	ldr	r3, [r0, #40]	@ 0x28
     6ee:	6898      	ldr	r0, [r3, #8]
     6f0:	8a80      	ldrh	r0, [r0, #20]
     6f2:	0a04      	lsrs	r4, r0, #8
     6f4:	704c      	strb	r4, [r1, #1]
     6f6:	7088      	strb	r0, [r1, #2]
     6f8:	24ff      	movs	r4, #255	@ 0xff
     6fa:	2000      	movs	r0, #0
     6fc:	b285      	uxth	r5, r0
     6fe:	42aa      	cmp	r2, r5
     700:	d81b      	bhi.n	0x73a
     702:	548c      	strb	r4, [r1, r2]
     704:	63d9      	str	r1, [r3, #60]	@ 0x3c
     706:	f3bf 8f4f 	dsb	sy
     70a:	f3bf 8f6f 	isb	sy
     70e:	b672      	cpsid	i
     710:	7eda      	ldrb	r2, [r3, #27]
     712:	2a01      	cmp	r2, #1
     714:	d10f      	bne.n	0x736
     716:	2106      	movs	r1, #6
     718:	76d9      	strb	r1, [r3, #27]
     71a:	68d9      	ldr	r1, [r3, #12]
     71c:	6808      	ldr	r0, [r1, #0]
     71e:	2102      	movs	r1, #2
     720:	6041      	str	r1, [r0, #4]
     722:	6042      	str	r2, [r0, #4]
     724:	7e99      	ldrb	r1, [r3, #26]
     726:	0003      	movs	r3, r0
     728:	008c      	lsls	r4, r1, #2
     72a:	3318      	adds	r3, #24
     72c:	191b      	adds	r3, r3, r4
     72e:	601a      	str	r2, [r3, #0]
     730:	3202      	adds	r2, #2
     732:	f7ff ffc9 	bl	0x6c8
     736:	b662      	cpsie	i
     738:	bd70      	pop	{r4, r5, r6, pc}
     73a:	5c0d      	ldrb	r5, [r1, r0]
     73c:	3001      	adds	r0, #1
     73e:	406c      	eors	r4, r5
     740:	e7dc      	b.n	0x6fc
     742:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
     744:	0006      	movs	r6, r0
     746:	6a85      	ldr	r5, [r0, #40]	@ 0x28
     748:	f3bf 8f4f 	dsb	sy
     74c:	f3bf 8f6f 	isb	sy
     750:	b672      	cpsid	i
     752:	7eeb      	ldrb	r3, [r5, #27]
     754:	2b01      	cmp	r3, #1
     756:	d004      	beq.n	0x762
     758:	2b06      	cmp	r3, #6
     75a:	d12d      	bne.n	0x7b8
     75c:	6beb      	ldr	r3, [r5, #60]	@ 0x3c
     75e:	2b00      	cmp	r3, #0
     760:	d12a      	bne.n	0x7b8
     762:	68eb      	ldr	r3, [r5, #12]
     764:	7e29      	ldrb	r1, [r5, #24]
     766:	681c      	ldr	r4, [r3, #0]
     768:	2280      	movs	r2, #128	@ 0x80
     76a:	0020      	movs	r0, r4
     76c:	f7ff ff9c 	bl	0x6a8
     770:	7eaf      	ldrb	r7, [r5, #26]
     772:	0020      	movs	r0, r4
     774:	0039      	movs	r1, r7
     776:	2202      	movs	r2, #2
     778:	f7ff ffa6 	bl	0x6c8
     77c:	00bf      	lsls	r7, r7, #2
     77e:	4b10      	ldr	r3, [pc, #64]	@ (0x7c0)
     780:	3418      	adds	r4, #24
     782:	19e4      	adds	r4, r4, r7
     784:	6023      	str	r3, [r4, #0]
     786:	2300      	movs	r3, #0
     788:	76eb      	strb	r3, [r5, #27]
     78a:	b662      	cpsie	i
     78c:	bf30      	wfi
     78e:	3648      	adds	r6, #72	@ 0x48
     790:	7833      	ldrb	r3, [r6, #0]
     792:	2b03      	cmp	r3, #3
     794:	d108      	bne.n	0x7a8
     796:	f3bf 8f4f 	dsb	sy
     79a:	f3bf 8f6f 	isb	sy
     79e:	b672      	cpsid	i
     7a0:	2380      	movs	r3, #128	@ 0x80
     7a2:	4a08      	ldr	r2, [pc, #32]	@ (0x7c4)
     7a4:	055b      	lsls	r3, r3, #21
     7a6:	601a      	str	r2, [r3, #0]
     7a8:	f3bf 8f4f 	dsb	sy
     7ac:	4b06      	ldr	r3, [pc, #24]	@ (0x7c8)
     7ae:	4a07      	ldr	r2, [pc, #28]	@ (0x7cc)
     7b0:	60da      	str	r2, [r3, #12]
     7b2:	f3bf 8f4f 	dsb	sy
     7b6:	e7fe      	b.n	0x7b6
     7b8:	b662      	cpsie	i
     7ba:	bf30      	wfi
     7bc:	e7c4      	b.n	0x748
     7be:	46c0      	nop			@ (mov r8, r8)
     7c0:	fffe 0000 	vaddl.u<illegal width 64>	q8, d14, d0
     7c4:	b055      	add	sp, #340	@ 0x154
     7c6:	5e1f      	ldrsh	r7, [r3, r0]
     7c8:	ed00 e000 	stc	0, cr14, [r0, #-0]
     7cc:	0004      	movs	r4, r0
     7ce:	05fa      	lsls	r2, r7, #23
     7d0:	b570      	push	{r4, r5, r6, lr}
     7d2:	0005      	movs	r5, r0
     7d4:	68c3      	ldr	r3, [r0, #12]
     7d6:	7e01      	ldrb	r1, [r0, #24]
     7d8:	681c      	ldr	r4, [r3, #0]
     7da:	2281      	movs	r2, #129	@ 0x81
     7dc:	0020      	movs	r0, r4
     7de:	f7ff ff63 	bl	0x6a8
     7e2:	7eae      	ldrb	r6, [r5, #26]
     7e4:	0020      	movs	r0, r4
     7e6:	0031      	movs	r1, r6
     7e8:	2201      	movs	r2, #1
     7ea:	f7ff ff6d 	bl	0x6c8
     7ee:	2302      	movs	r3, #2
     7f0:	6063      	str	r3, [r4, #4]
     7f2:	3b01      	subs	r3, #1
     7f4:	6063      	str	r3, [r4, #4]
     7f6:	00b6      	lsls	r6, r6, #2
     7f8:	4b06      	ldr	r3, [pc, #24]	@ (0x814)
     7fa:	3418      	adds	r4, #24
     7fc:	19a6      	adds	r6, r4, r6
     7fe:	6033      	str	r3, [r6, #0]
     800:	7e6b      	ldrb	r3, [r5, #25]
     802:	009b      	lsls	r3, r3, #2
     804:	18e4      	adds	r4, r4, r3
     806:	4b04      	ldr	r3, [pc, #16]	@ (0x818)
     808:	6023      	str	r3, [r4, #0]
     80a:	2300      	movs	r3, #0
     80c:	76eb      	strb	r3, [r5, #27]
     80e:	62eb      	str	r3, [r5, #44]	@ 0x2c
     810:	bd70      	pop	{r4, r5, r6, pc}
     812:	46c0      	nop			@ (mov r8, r8)
     814:	1117      	asrs	r7, r2, #4
     816:	0000      	movs	r0, r0
     818:	ffff 0000 	vaddl.u<illegal width 64>	q8, d15, d0
     81c:	2380      	movs	r3, #128	@ 0x80
     81e:	3801      	subs	r0, #1
     820:	045b      	lsls	r3, r3, #17
     822:	4298      	cmp	r0, r3
     824:	d20d      	bcs.n	0x842
     826:	21c0      	movs	r1, #192	@ 0xc0
     828:	4a06      	ldr	r2, [pc, #24]	@ (0x844)
     82a:	0609      	lsls	r1, r1, #24
     82c:	6050      	str	r0, [r2, #4]
     82e:	4806      	ldr	r0, [pc, #24]	@ (0x848)
     830:	6a03      	ldr	r3, [r0, #32]
     832:	021b      	lsls	r3, r3, #8
     834:	0a1b      	lsrs	r3, r3, #8
     836:	430b      	orrs	r3, r1
     838:	6203      	str	r3, [r0, #32]
     83a:	2300      	movs	r3, #0
     83c:	6093      	str	r3, [r2, #8]
     83e:	3307      	adds	r3, #7
     840:	6013      	str	r3, [r2, #0]
     842:	4770      	bx	lr
     844:	e010      	b.n	0x868
     846:	e000      	b.n	0x84a
     848:	ed00 e000 	stc	0, cr14, [r0, #-0]
     84c:	2080      	movs	r0, #128	@ 0x80
     84e:	b570      	push	{r4, r5, r6, lr}
     850:	0440      	lsls	r0, r0, #17
     852:	f7ff ffe3 	bl	0x81c
     856:	f000 f95b 	bl	0xb10
     85a:	211f      	movs	r1, #31
     85c:	2201      	movs	r2, #1
     85e:	4b0a      	ldr	r3, [pc, #40]	@ (0x888)
     860:	2480      	movs	r4, #128	@ 0x80
     862:	681b      	ldr	r3, [r3, #0]
     864:	4d09      	ldr	r5, [pc, #36]	@ (0x88c)
     866:	4019      	ands	r1, r3
     868:	408a      	lsls	r2, r1
     86a:	065b      	lsls	r3, r3, #25
     86c:	4908      	ldr	r1, [pc, #32]	@ (0x890)
     86e:	0f9b      	lsrs	r3, r3, #30
     870:	009b      	lsls	r3, r3, #2
     872:	5859      	ldr	r1, [r3, r1]
     874:	03e4      	lsls	r4, r4, #15
     876:	0090      	lsls	r0, r2, #2
     878:	68ab      	ldr	r3, [r5, #8]
     87a:	4023      	ands	r3, r4
     87c:	425e      	negs	r6, r3
     87e:	4173      	adcs	r3, r6
     880:	425b      	negs	r3, r3
     882:	4013      	ands	r3, r2
     884:	5043      	str	r3, [r0, r1]
     886:	e7f7      	b.n	0x878
     888:	0014      	movs	r4, r2
     88a:	1000      	asrs	r0, r0, #32
     88c:	e010      	b.n	0x8b0
     88e:	e000      	b.n	0x892
     890:	2ff8      	cmp	r7, #248	@ 0xf8
     892:	0000      	movs	r0, r0
     894:	b5f0      	push	{r4, r5, r6, r7, lr}
     896:	0643      	lsls	r3, r0, #25
     898:	b085      	sub	sp, #20
     89a:	0f9b      	lsrs	r3, r3, #30
     89c:	9301      	str	r3, [sp, #4]
     89e:	4b7d      	ldr	r3, [pc, #500]	@ (0xa94)
     8a0:	25f0      	movs	r5, #240	@ 0xf0
     8a2:	9303      	str	r3, [sp, #12]
     8a4:	9b01      	ldr	r3, [sp, #4]
     8a6:	9a03      	ldr	r2, [sp, #12]
     8a8:	009b      	lsls	r3, r3, #2
     8aa:	589a      	ldr	r2, [r3, r2]
     8ac:	231f      	movs	r3, #31
     8ae:	4003      	ands	r3, r0
     8b0:	9302      	str	r3, [sp, #8]
     8b2:	2301      	movs	r3, #1
     8b4:	001e      	movs	r6, r3
     8b6:	469c      	mov	ip, r3
     8b8:	9b02      	ldr	r3, [sp, #8]
     8ba:	27c0      	movs	r7, #192	@ 0xc0
     8bc:	409e      	lsls	r6, r3
     8be:	2380      	movs	r3, #128	@ 0x80
     8c0:	050c      	lsls	r4, r1, #20
     8c2:	022d      	lsls	r5, r5, #8
     8c4:	400d      	ands	r5, r1
     8c6:	0d24      	lsrs	r4, r4, #20
     8c8:	1489      	asrs	r1, r1, #18
     8ca:	b2b6      	uxth	r6, r6
     8cc:	01ff      	lsls	r7, r7, #7
     8ce:	021b      	lsls	r3, r3, #8
     8d0:	2900      	cmp	r1, #0
     8d2:	d111      	bne.n	0x8f8
     8d4:	4661      	mov	r1, ip
     8d6:	42bd      	cmp	r5, r7
     8d8:	d11d      	bne.n	0x916
     8da:	58d5      	ldr	r5, [r2, r3]
     8dc:	4335      	orrs	r5, r6
     8de:	50d5      	str	r5, [r2, r3]
     8e0:	05c3      	lsls	r3, r0, #23
     8e2:	d500      	bpl.n	0x8e6
     8e4:	e0ca      	b.n	0xa7c
     8e6:	1243      	asrs	r3, r0, #9
     8e8:	221f      	movs	r2, #31
     8ea:	001d      	movs	r5, r3
     8ec:	4015      	ands	r5, r2
     8ee:	42a9      	cmp	r1, r5
     8f0:	d000      	beq.n	0x8f4
     8f2:	e0c8      	b.n	0xa86
     8f4:	2200      	movs	r2, #0
     8f6:	e045      	b.n	0x984
     8f8:	b209      	sxth	r1, r1
     8fa:	42bd      	cmp	r5, r7
     8fc:	d10b      	bne.n	0x916
     8fe:	58d5      	ldr	r5, [r2, r3]
     900:	4335      	orrs	r5, r6
     902:	50d5      	str	r5, [r2, r3]
     904:	290c      	cmp	r1, #12
     906:	d000      	beq.n	0x90a
     908:	e070      	b.n	0x9ec
     90a:	4b63      	ldr	r3, [pc, #396]	@ (0xa98)
     90c:	4298      	cmp	r0, r3
     90e:	d15d      	bne.n	0x9cc
     910:	4b62      	ldr	r3, [pc, #392]	@ (0xa9c)
     912:	2200      	movs	r2, #0
     914:	e077      	b.n	0xa06
     916:	27e0      	movs	r7, #224	@ 0xe0
     918:	01ff      	lsls	r7, r7, #7
     91a:	42bd      	cmp	r5, r7
     91c:	d104      	bne.n	0x928
     91e:	58d1      	ldr	r1, [r2, r3]
     920:	4331      	orrs	r1, r6
     922:	50d1      	str	r1, [r2, r3]
     924:	2106      	movs	r1, #6
     926:	e7db      	b.n	0x8e0
     928:	58d7      	ldr	r7, [r2, r3]
     92a:	43b7      	bics	r7, r6
     92c:	50d7      	str	r7, [r2, r3]
     92e:	2380      	movs	r3, #128	@ 0x80
     930:	019b      	lsls	r3, r3, #6
     932:	429d      	cmp	r5, r3
     934:	d136      	bne.n	0x9a4
     936:	4b5a      	ldr	r3, [pc, #360]	@ (0xaa0)
     938:	4298      	cmp	r0, r3
     93a:	d109      	bne.n	0x950
     93c:	2200      	movs	r2, #0
     93e:	2107      	movs	r1, #7
     940:	4b58      	ldr	r3, [pc, #352]	@ (0xaa4)
     942:	601a      	str	r2, [r3, #0]
     944:	4b58      	ldr	r3, [pc, #352]	@ (0xaa8)
     946:	045a      	lsls	r2, r3, #17
     948:	0ed2      	lsrs	r2, r2, #27
     94a:	4291      	cmp	r1, r2
     94c:	d148      	bne.n	0x9e0
     94e:	e05b      	b.n	0xa08
     950:	4b56      	ldr	r3, [pc, #344]	@ (0xaac)
     952:	4298      	cmp	r0, r3
     954:	d105      	bne.n	0x962
     956:	2201      	movs	r2, #1
     958:	4b52      	ldr	r3, [pc, #328]	@ (0xaa4)
     95a:	2107      	movs	r1, #7
     95c:	601a      	str	r2, [r3, #0]
     95e:	4b54      	ldr	r3, [pc, #336]	@ (0xab0)
     960:	e7f1      	b.n	0x946
     962:	4b54      	ldr	r3, [pc, #336]	@ (0xab4)
     964:	4298      	cmp	r0, r3
     966:	d105      	bne.n	0x974
     968:	2200      	movs	r2, #0
     96a:	4b53      	ldr	r3, [pc, #332]	@ (0xab8)
     96c:	2107      	movs	r1, #7
     96e:	601a      	str	r2, [r3, #0]
     970:	4b52      	ldr	r3, [pc, #328]	@ (0xabc)
     972:	e7e8      	b.n	0x946
     974:	4b52      	ldr	r3, [pc, #328]	@ (0xac0)
     976:	2107      	movs	r1, #7
     978:	4298      	cmp	r0, r3
     97a:	d1b1      	bne.n	0x8e0
     97c:	4b4e      	ldr	r3, [pc, #312]	@ (0xab8)
     97e:	2201      	movs	r2, #1
     980:	601a      	str	r2, [r3, #0]
     982:	2201      	movs	r2, #1
     984:	230c      	movs	r3, #12
     986:	9901      	ldr	r1, [sp, #4]
     988:	4322      	orrs	r2, r4
     98a:	434b      	muls	r3, r1
     98c:	9902      	ldr	r1, [sp, #8]
     98e:	185b      	adds	r3, r3, r1
     990:	9903      	ldr	r1, [sp, #12]
     992:	005b      	lsls	r3, r3, #1
     994:	18cb      	adds	r3, r1, r3
     996:	8c1b      	ldrh	r3, [r3, #32]
     998:	494a      	ldr	r1, [pc, #296]	@ (0xac4)
     99a:	185b      	adds	r3, r3, r1
     99c:	009b      	lsls	r3, r3, #2
     99e:	601a      	str	r2, [r3, #0]
     9a0:	b005      	add	sp, #20
     9a2:	bdf0      	pop	{r4, r5, r6, r7, pc}
     9a4:	23c0      	movs	r3, #192	@ 0xc0
     9a6:	019b      	lsls	r3, r3, #6
     9a8:	429d      	cmp	r5, r3
     9aa:	d06a      	beq.n	0xa82
     9ac:	290f      	cmp	r1, #15
     9ae:	d1a9      	bne.n	0x904
     9b0:	2300      	movs	r3, #0
     9b2:	4a45      	ldr	r2, [pc, #276]	@ (0xac8)
     9b4:	009d      	lsls	r5, r3, #2
     9b6:	58ad      	ldr	r5, [r5, r2]
     9b8:	42a8      	cmp	r0, r5
     9ba:	d102      	bne.n	0x9c2
     9bc:	4a43      	ldr	r2, [pc, #268]	@ (0xacc)
     9be:	6013      	str	r3, [r2, #0]
     9c0:	e78e      	b.n	0x8e0
     9c2:	3301      	adds	r3, #1
     9c4:	2b04      	cmp	r3, #4
     9c6:	d1f5      	bne.n	0x9b4
     9c8:	f7ff ff40 	bl	0x84c
     9cc:	4b40      	ldr	r3, [pc, #256]	@ (0xad0)
     9ce:	4298      	cmp	r0, r3
     9d0:	d000      	beq.n	0x9d4
     9d2:	e785      	b.n	0x8e0
     9d4:	2201      	movs	r2, #1
     9d6:	4b31      	ldr	r3, [pc, #196]	@ (0xa9c)
     9d8:	601a      	str	r2, [r3, #0]
     9da:	2380      	movs	r3, #128	@ 0x80
     9dc:	431c      	orrs	r4, r3
     9de:	4b3d      	ldr	r3, [pc, #244]	@ (0xad4)
     9e0:	031b      	lsls	r3, r3, #12
     9e2:	0edb      	lsrs	r3, r3, #27
     9e4:	428b      	cmp	r3, r1
     9e6:	d1ef      	bne.n	0x9c8
     9e8:	2203      	movs	r2, #3
     9ea:	e7cb      	b.n	0x984
     9ec:	290d      	cmp	r1, #13
     9ee:	d10d      	bne.n	0xa0c
     9f0:	4b39      	ldr	r3, [pc, #228]	@ (0xad8)
     9f2:	4298      	cmp	r0, r3
     9f4:	d101      	bne.n	0x9fa
     9f6:	4b39      	ldr	r3, [pc, #228]	@ (0xadc)
     9f8:	e78b      	b.n	0x912
     9fa:	4b39      	ldr	r3, [pc, #228]	@ (0xae0)
     9fc:	4298      	cmp	r0, r3
     9fe:	d000      	beq.n	0xa02
     a00:	e76e      	b.n	0x8e0
     a02:	2201      	movs	r2, #1
     a04:	4b35      	ldr	r3, [pc, #212]	@ (0xadc)
     a06:	601a      	str	r2, [r3, #0]
     a08:	2202      	movs	r2, #2
     a0a:	e7bb      	b.n	0x984
     a0c:	290e      	cmp	r1, #14
     a0e:	d11d      	bne.n	0xa4c
     a10:	4b34      	ldr	r3, [pc, #208]	@ (0xae4)
     a12:	4298      	cmp	r0, r3
     a14:	d101      	bne.n	0xa1a
     a16:	4b34      	ldr	r3, [pc, #208]	@ (0xae8)
     a18:	e77b      	b.n	0x912
     a1a:	4b34      	ldr	r3, [pc, #208]	@ (0xaec)
     a1c:	4298      	cmp	r0, r3
     a1e:	d101      	bne.n	0xa24
     a20:	4b31      	ldr	r3, [pc, #196]	@ (0xae8)
     a22:	e7ac      	b.n	0x97e
     a24:	4b32      	ldr	r3, [pc, #200]	@ (0xaf0)
     a26:	4298      	cmp	r0, r3
     a28:	d102      	bne.n	0xa30
     a2a:	2202      	movs	r2, #2
     a2c:	4b2e      	ldr	r3, [pc, #184]	@ (0xae8)
     a2e:	e7ea      	b.n	0xa06
     a30:	4b30      	ldr	r3, [pc, #192]	@ (0xaf4)
     a32:	4298      	cmp	r0, r3
     a34:	d101      	bne.n	0xa3a
     a36:	4b30      	ldr	r3, [pc, #192]	@ (0xaf8)
     a38:	e76b      	b.n	0x912
     a3a:	4b30      	ldr	r3, [pc, #192]	@ (0xafc)
     a3c:	4298      	cmp	r0, r3
     a3e:	d000      	beq.n	0xa42
     a40:	e74e      	b.n	0x8e0
     a42:	2201      	movs	r2, #1
     a44:	4b2c      	ldr	r3, [pc, #176]	@ (0xaf8)
     a46:	601a      	str	r2, [r3, #0]
     a48:	4b2d      	ldr	r3, [pc, #180]	@ (0xb00)
     a4a:	e7c9      	b.n	0x9e0
     a4c:	290b      	cmp	r1, #11
     a4e:	d10f      	bne.n	0xa70
     a50:	4b11      	ldr	r3, [pc, #68]	@ (0xa98)
     a52:	4298      	cmp	r0, r3
     a54:	d104      	bne.n	0xa60
     a56:	2200      	movs	r2, #0
     a58:	4b2a      	ldr	r3, [pc, #168]	@ (0xb04)
     a5a:	619a      	str	r2, [r3, #24]
     a5c:	4b2a      	ldr	r3, [pc, #168]	@ (0xb08)
     a5e:	e7bf      	b.n	0x9e0
     a60:	4b2a      	ldr	r3, [pc, #168]	@ (0xb0c)
     a62:	4298      	cmp	r0, r3
     a64:	d000      	beq.n	0xa68
     a66:	e73b      	b.n	0x8e0
     a68:	2201      	movs	r2, #1
     a6a:	4b26      	ldr	r3, [pc, #152]	@ (0xb04)
     a6c:	619a      	str	r2, [r3, #24]
     a6e:	e7cb      	b.n	0xa08
     a70:	05c3      	lsls	r3, r0, #23
     a72:	d400      	bmi.n	0xa76
     a74:	e737      	b.n	0x8e6
     a76:	2902      	cmp	r1, #2
     a78:	d100      	bne.n	0xa7c
     a7a:	e734      	b.n	0x8e6
     a7c:	2380      	movs	r3, #128	@ 0x80
     a7e:	431c      	orrs	r4, r3
     a80:	e731      	b.n	0x8e6
     a82:	2102      	movs	r1, #2
     a84:	e72f      	b.n	0x8e6
     a86:	1380      	asrs	r0, r0, #14
     a88:	4010      	ands	r0, r2
     a8a:	4288      	cmp	r0, r1
     a8c:	d000      	beq.n	0xa90
     a8e:	e75a      	b.n	0x946
     a90:	e777      	b.n	0x982
     a92:	46c0      	nop			@ (mov r8, r8)
     a94:	2ff8      	cmp	r7, #248	@ 0xf8
     a96:	0000      	movs	r0, r0
     a98:	4242      	negs	r2, r0
     a9a:	0065      	lsls	r5, r4, #1
     a9c:	40c8      	lsrs	r0, r1
     a9e:	4004      	ands	r4, r0
     aa0:	c202      	stmia	r2!, {r1}
     aa2:	003a      	movs	r2, r7
     aa4:	40c0      	lsrs	r0, r0
     aa6:	4004      	ands	r4, r0
     aa8:	1d61      	adds	r1, r4, #5
     aaa:	0000      	movs	r0, r0
     aac:	8263      	strh	r3, [r4, #18]
     aae:	003d      	movs	r5, r7
     ab0:	1ec1      	subs	r1, r0, #3
     ab2:	0000      	movs	r0, r0
     ab4:	4225      	tst	r5, r4
     ab6:	003c      	movs	r4, r7
     ab8:	40d0      	lsrs	r0, r2
     aba:	4004      	ands	r4, r0
     abc:	1e21      	subs	r1, r4, #0
     abe:	0000      	movs	r0, r0
     ac0:	c249      	stmia	r2!, {r0, r3, r6}
     ac2:	0001      	movs	r1, r0
     ac4:	1000      	asrs	r0, r0, #32
     ac6:	1001      	asrs	r1, r0, #32
     ac8:	3008      	adds	r0, #8
     aca:	0000      	movs	r0, r0
     acc:	40d4      	lsrs	r4, r2
     ace:	4004      	ands	r4, r0
     ad0:	832a      	strh	r2, [r5, #24]
     ad2:	0c30      	lsrs	r0, r6, #16
     ad4:	1841      	adds	r1, r0, r1
     ad6:	0006      	movs	r6, r0
     ad8:	8243      	strh	r3, [r0, #18]
     ada:	006d      	lsls	r5, r5, #1
     adc:	40cc      	lsrs	r4, r1
     ade:	4004      	ands	r4, r0
     ae0:	8229      	strh	r1, [r5, #16]
     ae2:	0069      	lsls	r1, r5, #1
     ae4:	4a0a      	ldr	r2, [pc, #40]	@ (0xb10)
     ae6:	0670      	lsls	r0, r6, #25
     ae8:	40b0      	lsls	r0, r6
     aea:	4004      	ands	r4, r0
     aec:	824b      	strh	r3, [r1, #18]
     aee:	003b      	movs	r3, r7
     af0:	0206      	lsls	r6, r0, #8
     af2:	0070      	lsls	r0, r6, #1
     af4:	c241      	stmia	r2!, {r0, r6}
     af6:	0074      	lsls	r4, r6, #1
     af8:	40c4      	lsrs	r4, r0
     afa:	4004      	ands	r4, r0
     afc:	4262      	negs	r2, r4
     afe:	0e35      	lsrs	r5, r6, #24
     b00:	1aa1      	subs	r1, r4, r2
     b02:	0007      	movs	r7, r0
     b04:	4000      	ands	r0, r0
     b06:	4004      	ands	r4, r0
     b08:	32a1      	adds	r2, #161	@ 0xa1
     b0a:	0000      	movs	r0, r0
     b0c:	8244      	strh	r4, [r0, #18]
     b0e:	0059      	lsls	r1, r3, #1
     b10:	21c0      	movs	r1, #192	@ 0xc0
     b12:	b510      	push	{r4, lr}
     b14:	4c05      	ldr	r4, [pc, #20]	@ (0xb2c)
     b16:	01c9      	lsls	r1, r1, #7
     b18:	6820      	ldr	r0, [r4, #0]
     b1a:	f7ff febb 	bl	0x894
     b1e:	2180      	movs	r1, #128	@ 0x80
     b20:	6860      	ldr	r0, [r4, #4]
     b22:	0149      	lsls	r1, r1, #5
     b24:	f7ff feb6 	bl	0x894
     b28:	bd10      	pop	{r4, pc}
     b2a:	46c0      	nop			@ (mov r8, r8)
     b2c:	0014      	movs	r4, r2
     b2e:	1000      	asrs	r0, r0, #32
     b30:	2110      	movs	r1, #16
     b32:	b5f0      	push	{r4, r5, r6, r7, lr}
     b34:	7e03      	ldrb	r3, [r0, #24]
     b36:	68c2      	ldr	r2, [r0, #12]
     b38:	4099      	lsls	r1, r3
     b3a:	000d      	movs	r5, r1
     b3c:	6812      	ldr	r2, [r2, #0]
     b3e:	0004      	movs	r4, r0
     b40:	6810      	ldr	r0, [r2, #0]
     b42:	b085      	sub	sp, #20
     b44:	4005      	ands	r5, r0
     b46:	9500      	str	r5, [sp, #0]
     b48:	4201      	tst	r1, r0
     b4a:	d026      	beq.n	0xb9a
     b4c:	0098      	lsls	r0, r3, #2
     b4e:	0013      	movs	r3, r2
     b50:	0015      	movs	r5, r2
     b52:	332c      	adds	r3, #44	@ 0x2c
     b54:	181b      	adds	r3, r3, r0
     b56:	6818      	ldr	r0, [r3, #0]
     b58:	7ea3      	ldrb	r3, [r4, #26]
     b5a:	3518      	adds	r5, #24
     b5c:	009b      	lsls	r3, r3, #2
     b5e:	18eb      	adds	r3, r5, r3
     b60:	681e      	ldr	r6, [r3, #0]
     b62:	7e63      	ldrb	r3, [r4, #25]
     b64:	009b      	lsls	r3, r3, #2
     b66:	18ed      	adds	r5, r5, r3
     b68:	682b      	ldr	r3, [r5, #0]
     b6a:	4283      	cmp	r3, r0
     b6c:	d915      	bls.n	0xb9a
     b6e:	6923      	ldr	r3, [r4, #16]
     b70:	4db9      	ldr	r5, [pc, #740]	@ (0xe58)
     b72:	065f      	lsls	r7, r3, #25
     b74:	0fbf      	lsrs	r7, r7, #30
     b76:	00bf      	lsls	r7, r7, #2
     b78:	597f      	ldr	r7, [r7, r5]
     b7a:	251f      	movs	r5, #31
     b7c:	402b      	ands	r3, r5
     b7e:	3d1b      	subs	r5, #27
     b80:	409d      	lsls	r5, r3
     b82:	59eb      	ldr	r3, [r5, r7]
     b84:	2b00      	cmp	r3, #0
     b86:	d002      	beq.n	0xb8e
     b88:	6011      	str	r1, [r2, #0]
     b8a:	b005      	add	sp, #20
     b8c:	bdf0      	pop	{r4, r5, r6, r7, pc}
     b8e:	6893      	ldr	r3, [r2, #8]
     b90:	4298      	cmp	r0, r3
     b92:	d817      	bhi.n	0xbc4
     b94:	1a1b      	subs	r3, r3, r0
     b96:	2b03      	cmp	r3, #3
     b98:	d9f3      	bls.n	0xb82
     b9a:	7ee0      	ldrb	r0, [r4, #27]
     b9c:	280d      	cmp	r0, #13
     b9e:	d901      	bls.n	0xba4
     ba0:	f000 fc78 	bl	0x1494
     ba4:	f002 f874 	bl	0x2c90
     ba8:	0011      	movs	r1, r2
     baa:	0034      	movs	r4, r6
     bac:	0037      	movs	r7, r6
     bae:	0041      	lsls	r1, r0, #1
     bb0:	018e      	lsls	r6, r1, #6
     bb2:	022b      	lsls	r3, r5, #8
     bb4:	0255      	lsls	r5, r2, #9
     bb6:	02f0      	lsls	r0, r6, #11
     bb8:	032e      	lsls	r6, r5, #12
     bba:	0346      	lsls	r6, r0, #13
     bbc:	03f0      	lsls	r0, r6, #15
     bbe:	0412      	lsls	r2, r2, #16
     bc0:	0446      	lsls	r6, r0, #17
     bc2:	0455      	lsls	r5, r2, #17
     bc4:	3301      	adds	r3, #1
     bc6:	199b      	adds	r3, r3, r6
     bc8:	e7e4      	b.n	0xb94
     bca:	68e3      	ldr	r3, [r4, #12]
     bcc:	7ea1      	ldrb	r1, [r4, #26]
     bce:	681d      	ldr	r5, [r3, #0]
     bd0:	2301      	movs	r3, #1
     bd2:	408b      	lsls	r3, r1
     bd4:	682a      	ldr	r2, [r5, #0]
     bd6:	421a      	tst	r2, r3
     bd8:	d106      	bne.n	0xbe8
     bda:	2305      	movs	r3, #5
     bdc:	60ab      	str	r3, [r5, #8]
     bde:	22ff      	movs	r2, #255	@ 0xff
     be0:	68e3      	ldr	r3, [r4, #12]
     be2:	681b      	ldr	r3, [r3, #0]
     be4:	601a      	str	r2, [r3, #0]
     be6:	e7d0      	b.n	0xb8a
     be8:	002b      	movs	r3, r5
     bea:	008a      	lsls	r2, r1, #2
     bec:	3318      	adds	r3, #24
     bee:	189b      	adds	r3, r3, r2
     bf0:	22a6      	movs	r2, #166	@ 0xa6
     bf2:	0152      	lsls	r2, r2, #5
     bf4:	601a      	str	r2, [r3, #0]
     bf6:	0028      	movs	r0, r5
     bf8:	2203      	movs	r2, #3
     bfa:	f7ff fd65 	bl	0x6c8
     bfe:	2306      	movs	r3, #6
     c00:	7e21      	ldrb	r1, [r4, #24]
     c02:	76e3      	strb	r3, [r4, #27]
     c04:	330a      	adds	r3, #10
     c06:	408b      	lsls	r3, r1
     c08:	682a      	ldr	r2, [r5, #0]
     c0a:	421a      	tst	r2, r3
     c0c:	d1c5      	bne.n	0xb9a
     c0e:	e7e6      	b.n	0xbde
     c10:	9b00      	ldr	r3, [sp, #0]
     c12:	2b00      	cmp	r3, #0
     c14:	d0e3      	beq.n	0xbde
     c16:	0022      	movs	r2, r4
     c18:	2300      	movs	r3, #0
     c1a:	3258      	adds	r2, #88	@ 0x58
     c1c:	6323      	str	r3, [r4, #48]	@ 0x30
     c1e:	8013      	strh	r3, [r2, #0]
     c20:	22ff      	movs	r2, #255	@ 0xff
     c22:	62e3      	str	r3, [r4, #44]	@ 0x2c
     c24:	3301      	adds	r3, #1
     c26:	6562      	str	r2, [r4, #84]	@ 0x54
     c28:	6523      	str	r3, [r4, #80]	@ 0x50
     c2a:	68e3      	ldr	r3, [r4, #12]
     c2c:	7ea1      	ldrb	r1, [r4, #26]
     c2e:	681d      	ldr	r5, [r3, #0]
     c30:	008b      	lsls	r3, r1, #2
     c32:	002e      	movs	r6, r5
     c34:	3618      	adds	r6, #24
     c36:	18f6      	adds	r6, r6, r3
     c38:	4b88      	ldr	r3, [pc, #544]	@ (0xe5c)
     c3a:	6837      	ldr	r7, [r6, #0]
     c3c:	2203      	movs	r2, #3
     c3e:	6033      	str	r3, [r6, #0]
     c40:	0028      	movs	r0, r5
     c42:	f7ff fd41 	bl	0x6c8
     c46:	9b00      	ldr	r3, [sp, #0]
     c48:	2b00      	cmp	r3, #0
     c4a:	d000      	beq.n	0xc4e
     c4c:	e11c      	b.n	0xe88
     c4e:	6d63      	ldr	r3, [r4, #84]	@ 0x54
     c50:	6b25      	ldr	r5, [r4, #48]	@ 0x30
     c52:	2b00      	cmp	r3, #0
     c54:	d135      	bne.n	0xcc2
     c56:	6d23      	ldr	r3, [r4, #80]	@ 0x50
     c58:	2b00      	cmp	r3, #0
     c5a:	d139      	bne.n	0xcd0
     c5c:	2300      	movs	r3, #0
     c5e:	62e3      	str	r3, [r4, #44]	@ 0x2c
     c60:	3306      	adds	r3, #6
     c62:	76e3      	strb	r3, [r4, #27]
     c64:	0022      	movs	r2, r4
     c66:	0021      	movs	r1, r4
     c68:	6b25      	ldr	r5, [r4, #48]	@ 0x30
     c6a:	325c      	adds	r2, #92	@ 0x5c
     c6c:	3158      	adds	r1, #88	@ 0x58
     c6e:	7810      	ldrb	r0, [r2, #0]
     c70:	880b      	ldrh	r3, [r1, #0]
     c72:	2d01      	cmp	r5, #1
     c74:	d138      	bne.n	0xce8
     c76:	2800      	cmp	r0, #0
     c78:	d100      	bne.n	0xc7c
     c7a:	e0f7      	b.n	0xe6c
     c7c:	2000      	movs	r0, #0
     c7e:	7010      	strb	r0, [r2, #0]
     c80:	2210      	movs	r2, #16
     c82:	4393      	bics	r3, r2
     c84:	6ce2      	ldr	r2, [r4, #76]	@ 0x4c
     c86:	800b      	strh	r3, [r1, #0]
     c88:	4282      	cmp	r2, r0
     c8a:	d002      	beq.n	0xc92
     c8c:	6b63      	ldr	r3, [r4, #52]	@ 0x34
     c8e:	2bcc      	cmp	r3, #204	@ 0xcc
     c90:	d013      	beq.n	0xcba
     c92:	69e1      	ldr	r1, [r4, #28]
     c94:	6a23      	ldr	r3, [r4, #32]
     c96:	4299      	cmp	r1, r3
     c98:	da09      	bge.n	0xcae
     c9a:	6a61      	ldr	r1, [r4, #36]	@ 0x24
     c9c:	6aa3      	ldr	r3, [r4, #40]	@ 0x28
     c9e:	4299      	cmp	r1, r3
     ca0:	da00      	bge.n	0xca4
     ca2:	e0ba      	b.n	0xe1a
     ca4:	2a00      	cmp	r2, #0
     ca6:	d002      	beq.n	0xcae
     ca8:	6b63      	ldr	r3, [r4, #52]	@ 0x34
     caa:	2bcc      	cmp	r3, #204	@ 0xcc
     cac:	d005      	beq.n	0xcba
     cae:	0022      	movs	r2, r4
     cb0:	2380      	movs	r3, #128	@ 0x80
     cb2:	325a      	adds	r2, #90	@ 0x5a
     cb4:	8811      	ldrh	r1, [r2, #0]
     cb6:	430b      	orrs	r3, r1
     cb8:	8013      	strh	r3, [r2, #0]
     cba:	0020      	movs	r0, r4
     cbc:	f7ff fb9f 	bl	0x3fe
     cc0:	e018      	b.n	0xcf4
     cc2:	0022      	movs	r2, r4
     cc4:	2310      	movs	r3, #16
     cc6:	3258      	adds	r2, #88	@ 0x58
     cc8:	8811      	ldrh	r1, [r2, #0]
     cca:	430b      	orrs	r3, r1
     ccc:	8013      	strh	r3, [r2, #0]
     cce:	e7c5      	b.n	0xc5c
     cd0:	9b00      	ldr	r3, [sp, #0]
     cd2:	62e3      	str	r3, [r4, #44]	@ 0x2c
     cd4:	2306      	movs	r3, #6
     cd6:	76e3      	strb	r3, [r4, #27]
     cd8:	2d07      	cmp	r5, #7
     cda:	ddc3      	ble.n	0xc64
     cdc:	222c      	movs	r2, #44	@ 0x2c
     cde:	6c23      	ldr	r3, [r4, #64]	@ 0x40
     ce0:	781b      	ldrb	r3, [r3, #0]
     ce2:	4393      	bics	r3, r2
     ce4:	2b90      	cmp	r3, #144	@ 0x90
     ce6:	d01f      	beq.n	0xd28
     ce8:	0022      	movs	r2, r4
     cea:	2380      	movs	r3, #128	@ 0x80
     cec:	3258      	adds	r2, #88	@ 0x58
     cee:	8811      	ldrh	r1, [r2, #0]
     cf0:	430b      	orrs	r3, r1
     cf2:	8013      	strh	r3, [r2, #0]
     cf4:	4e5a      	ldr	r6, [pc, #360]	@ (0xe60)
     cf6:	0022      	movs	r2, r4
     cf8:	325c      	adds	r2, #92	@ 0x5c
     cfa:	7813      	ldrb	r3, [r2, #0]
     cfc:	2b00      	cmp	r3, #0
     cfe:	d005      	beq.n	0xd0c
     d00:	0023      	movs	r3, r4
     d02:	2101      	movs	r1, #1
     d04:	335e      	adds	r3, #94	@ 0x5e
     d06:	7019      	strb	r1, [r3, #0]
     d08:	2300      	movs	r3, #0
     d0a:	7013      	strb	r3, [r2, #0]
     d0c:	68e3      	ldr	r3, [r4, #12]
     d0e:	2281      	movs	r2, #129	@ 0x81
     d10:	681d      	ldr	r5, [r3, #0]
     d12:	7e21      	ldrb	r1, [r4, #24]
     d14:	0028      	movs	r0, r5
     d16:	f7ff fcc7 	bl	0x6a8
     d1a:	7ea2      	ldrb	r2, [r4, #26]
     d1c:	3518      	adds	r5, #24
     d1e:	0092      	lsls	r2, r2, #2
     d20:	3e01      	subs	r6, #1
     d22:	18a8      	adds	r0, r5, r2
     d24:	6006      	str	r6, [r0, #0]
     d26:	e75a      	b.n	0xbde
     d28:	68a0      	ldr	r0, [r4, #8]
     d2a:	6803      	ldr	r3, [r0, #0]
     d2c:	6b5b      	ldr	r3, [r3, #52]	@ 0x34
     d2e:	4798      	blx	r3
     d30:	42a8      	cmp	r0, r5
     d32:	db97      	blt.n	0xc64
     d34:	6c23      	ldr	r3, [r4, #64]	@ 0x40
     d36:	7919      	ldrb	r1, [r3, #4]
     d38:	78da      	ldrb	r2, [r3, #3]
     d3a:	0209      	lsls	r1, r1, #8
     d3c:	4311      	orrs	r1, r2
     d3e:	795b      	ldrb	r3, [r3, #5]
     d40:	ba49      	rev16	r1, r1
     d42:	68a2      	ldr	r2, [r4, #8]
     d44:	b289      	uxth	r1, r1
     d46:	2b7f      	cmp	r3, #127	@ 0x7f
     d48:	d914      	bls.n	0xd74
     d4a:	424d      	negs	r5, r1
     d4c:	414d      	adcs	r5, r1
     d4e:	6b50      	ldr	r0, [r2, #52]	@ 0x34
     d50:	9b00      	ldr	r3, [sp, #0]
     d52:	b2ed      	uxtb	r5, r5
     d54:	2800      	cmp	r0, #0
     d56:	d004      	beq.n	0xd62
     d58:	6803      	ldr	r3, [r0, #0]
     d5a:	681b      	ldr	r3, [r3, #0]
     d5c:	4798      	blx	r3
     d5e:	43c3      	mvns	r3, r0
     d60:	0fdb      	lsrs	r3, r3, #31
     d62:	431d      	orrs	r5, r3
     d64:	68a3      	ldr	r3, [r4, #8]
     d66:	6b1b      	ldr	r3, [r3, #48]	@ 0x30
     d68:	7d1b      	ldrb	r3, [r3, #20]
     d6a:	075b      	lsls	r3, r3, #29
     d6c:	d506      	bpl.n	0xd7c
     d6e:	2d00      	cmp	r5, #0
     d70:	d0c0      	beq.n	0xcf4
     d72:	e003      	b.n	0xd7c
     d74:	8a93      	ldrh	r3, [r2, #20]
     d76:	9d00      	ldr	r5, [sp, #0]
     d78:	4299      	cmp	r1, r3
     d7a:	d1f3      	bne.n	0xd64
     d7c:	2220      	movs	r2, #32
     d7e:	6c20      	ldr	r0, [r4, #64]	@ 0x40
     d80:	7803      	ldrb	r3, [r0, #0]
     d82:	4213      	tst	r3, r2
     d84:	d00e      	beq.n	0xda4
     d86:	6863      	ldr	r3, [r4, #4]
     d88:	2b00      	cmp	r3, #0
     d8a:	d130      	bne.n	0xdee
     d8c:	22cc      	movs	r2, #204	@ 0xcc
     d8e:	62e2      	str	r2, [r4, #44]	@ 0x2c
     d90:	6b22      	ldr	r2, [r4, #48]	@ 0x30
     d92:	6c21      	ldr	r1, [r4, #64]	@ 0x40
     d94:	4293      	cmp	r3, r2
     d96:	db33      	blt.n	0xe00
     d98:	0023      	movs	r3, r4
     d9a:	6062      	str	r2, [r4, #4]
     d9c:	2200      	movs	r2, #0
     d9e:	3358      	adds	r3, #88	@ 0x58
     da0:	801a      	strh	r2, [r3, #0]
     da2:	e015      	b.n	0xdd0
     da4:	6821      	ldr	r1, [r4, #0]
     da6:	780d      	ldrb	r5, [r1, #0]
     da8:	406b      	eors	r3, r5
     daa:	4393      	bics	r3, r2
     dac:	d1eb      	bne.n	0xd86
     dae:	6b22      	ldr	r2, [r4, #48]	@ 0x30
     db0:	3a01      	subs	r2, #1
     db2:	3301      	adds	r3, #1
     db4:	4293      	cmp	r3, r2
     db6:	da04      	bge.n	0xdc2
     db8:	5cc6      	ldrb	r6, [r0, r3]
     dba:	5ccd      	ldrb	r5, [r1, r3]
     dbc:	42ae      	cmp	r6, r5
     dbe:	d0f8      	beq.n	0xdb2
     dc0:	e7e1      	b.n	0xd86
     dc2:	4293      	cmp	r3, r2
     dc4:	d1df      	bne.n	0xd86
     dc6:	6863      	ldr	r3, [r4, #4]
     dc8:	2b00      	cmp	r3, #0
     dca:	d110      	bne.n	0xdee
     dcc:	33cc      	adds	r3, #204	@ 0xcc
     dce:	62e3      	str	r3, [r4, #44]	@ 0x2c
     dd0:	68a3      	ldr	r3, [r4, #8]
     dd2:	2140      	movs	r1, #64	@ 0x40
     dd4:	6b1b      	ldr	r3, [r3, #48]	@ 0x30
     dd6:	7d1a      	ldrb	r2, [r3, #20]
     dd8:	2301      	movs	r3, #1
     dda:	0852      	lsrs	r2, r2, #1
     ddc:	4393      	bics	r3, r2
     dde:	6c22      	ldr	r2, [r4, #64]	@ 0x40
     de0:	7812      	ldrb	r2, [r2, #0]
     de2:	400a      	ands	r2, r1
     de4:	4313      	orrs	r3, r2
     de6:	d010      	beq.n	0xe0a
     de8:	2300      	movs	r3, #0
     dea:	62e3      	str	r3, [r4, #44]	@ 0x2c
     dec:	e782      	b.n	0xcf4
     dee:	2300      	movs	r3, #0
     df0:	0022      	movs	r2, r4
     df2:	62e3      	str	r3, [r4, #44]	@ 0x2c
     df4:	3258      	adds	r2, #88	@ 0x58
     df6:	8811      	ldrh	r1, [r2, #0]
     df8:	3340      	adds	r3, #64	@ 0x40
     dfa:	430b      	orrs	r3, r1
     dfc:	8013      	strh	r3, [r2, #0]
     dfe:	e7e7      	b.n	0xdd0
     e00:	5cc9      	ldrb	r1, [r1, r3]
     e02:	6822      	ldr	r2, [r4, #0]
     e04:	54d1      	strb	r1, [r2, r3]
     e06:	3301      	adds	r3, #1
     e08:	e7c2      	b.n	0xd90
     e0a:	6ae3      	ldr	r3, [r4, #44]	@ 0x2c
     e0c:	2b00      	cmp	r3, #0
     e0e:	d100      	bne.n	0xe12
     e10:	e770      	b.n	0xcf4
     e12:	2305      	movs	r3, #5
     e14:	4e13      	ldr	r6, [pc, #76]	@ (0xe64)
     e16:	76e3      	strb	r3, [r4, #27]
     e18:	e76d      	b.n	0xcf6
     e1a:	0023      	movs	r3, r4
     e1c:	0020      	movs	r0, r4
     e1e:	0021      	movs	r1, r4
     e20:	335a      	adds	r3, #90	@ 0x5a
     e22:	305d      	adds	r0, #93	@ 0x5d
     e24:	315e      	adds	r1, #94	@ 0x5e
     e26:	2a00      	cmp	r2, #0
     e28:	d00d      	beq.n	0xe46
     e2a:	6b62      	ldr	r2, [r4, #52]	@ 0x34
     e2c:	2ac0      	cmp	r2, #192	@ 0xc0
     e2e:	d001      	beq.n	0xe34
     e30:	2a00      	cmp	r2, #0
     e32:	d108      	bne.n	0xe46
     e34:	2210      	movs	r2, #16
     e36:	881d      	ldrh	r5, [r3, #0]
     e38:	4e0b      	ldr	r6, [pc, #44]	@ (0xe68)
     e3a:	432a      	orrs	r2, r5
     e3c:	801a      	strh	r2, [r3, #0]
     e3e:	2301      	movs	r3, #1
     e40:	7003      	strb	r3, [r0, #0]
     e42:	700b      	strb	r3, [r1, #0]
     e44:	e757      	b.n	0xcf6
     e46:	2204      	movs	r2, #4
     e48:	881d      	ldrh	r5, [r3, #0]
     e4a:	432a      	orrs	r2, r5
     e4c:	801a      	strh	r2, [r3, #0]
     e4e:	2300      	movs	r3, #0
     e50:	7003      	strb	r3, [r0, #0]
     e52:	3301      	adds	r3, #1
     e54:	700b      	strb	r3, [r1, #0]
     e56:	e74d      	b.n	0xcf4
     e58:	2ff8      	cmp	r7, #248	@ 0xf8
     e5a:	0000      	movs	r0, r0
     e5c:	fffe 0000 	vaddl.u<illegal width 64>	q8, d14, d0
     e60:	13f0      	asrs	r0, r6, #15
     e62:	0000      	movs	r0, r0
     e64:	05b3      	lsls	r3, r6, #22
     e66:	0000      	movs	r0, r0
     e68:	3ca1      	subs	r4, #161	@ 0xa1
     e6a:	0000      	movs	r0, r0
     e6c:	6b62      	ldr	r2, [r4, #52]	@ 0x34
     e6e:	2acc      	cmp	r2, #204	@ 0xcc
     e70:	d006      	beq.n	0xe80
     e72:	2a0c      	cmp	r2, #12
     e74:	d004      	beq.n	0xe80
     e76:	2ac0      	cmp	r2, #192	@ 0xc0
     e78:	d002      	beq.n	0xe80
     e7a:	2a00      	cmp	r2, #0
     e7c:	d000      	beq.n	0xe80
     e7e:	e733      	b.n	0xce8
     e80:	2290      	movs	r2, #144	@ 0x90
     e82:	4393      	bics	r3, r2
     e84:	800b      	strh	r3, [r1, #0]
     e86:	e735      	b.n	0xcf4
     e88:	002b      	movs	r3, r5
     e8a:	7e21      	ldrb	r1, [r4, #24]
     e8c:	332c      	adds	r3, #44	@ 0x2c
     e8e:	0088      	lsls	r0, r1, #2
     e90:	181b      	adds	r3, r3, r0
     e92:	68aa      	ldr	r2, [r5, #8]
     e94:	681b      	ldr	r3, [r3, #0]
     e96:	4293      	cmp	r3, r2
     e98:	d811      	bhi.n	0xebe
     e9a:	1ad3      	subs	r3, r2, r3
     e9c:	3302      	adds	r3, #2
     e9e:	60ab      	str	r3, [r5, #8]
     ea0:	4bcf      	ldr	r3, [pc, #828]	@ (0x11e0)
     ea2:	2281      	movs	r2, #129	@ 0x81
     ea4:	6033      	str	r3, [r6, #0]
     ea6:	0028      	movs	r0, r5
     ea8:	f7ff fbfe 	bl	0x6a8
     eac:	2304      	movs	r3, #4
     eae:	76e3      	strb	r3, [r4, #27]
     eb0:	2300      	movs	r3, #0
     eb2:	6363      	str	r3, [r4, #52]	@ 0x34
     eb4:	64a3      	str	r3, [r4, #72]	@ 0x48
     eb6:	3301      	adds	r3, #1
     eb8:	6463      	str	r3, [r4, #68]	@ 0x44
     eba:	64e3      	str	r3, [r4, #76]	@ 0x4c
     ebc:	e68f      	b.n	0xbde
     ebe:	3201      	adds	r2, #1
     ec0:	19d2      	adds	r2, r2, r7
     ec2:	e7ea      	b.n	0xe9a
     ec4:	2001      	movs	r0, #1
     ec6:	68e3      	ldr	r3, [r4, #12]
     ec8:	7ea2      	ldrb	r2, [r4, #26]
     eca:	681b      	ldr	r3, [r3, #0]
     ecc:	4090      	lsls	r0, r2
     ece:	681d      	ldr	r5, [r3, #0]
     ed0:	0029      	movs	r1, r5
     ed2:	4001      	ands	r1, r0
     ed4:	4205      	tst	r5, r0
     ed6:	d059      	beq.n	0xf8c
     ed8:	3318      	adds	r3, #24
     eda:	0092      	lsls	r2, r2, #2
     edc:	189b      	adds	r3, r3, r2
     ede:	681b      	ldr	r3, [r3, #0]
     ee0:	3301      	adds	r3, #1
     ee2:	6ca2      	ldr	r2, [r4, #72]	@ 0x48
     ee4:	0010      	movs	r0, r2
     ee6:	3044      	adds	r0, #68	@ 0x44
     ee8:	4298      	cmp	r0, r3
     eea:	da6e      	bge.n	0xfca
     eec:	2680      	movs	r6, #128	@ 0x80
     eee:	3268      	adds	r2, #104	@ 0x68
     ef0:	64a2      	str	r2, [r4, #72]	@ 0x48
     ef2:	0076      	lsls	r6, r6, #1
     ef4:	6ca2      	ldr	r2, [r4, #72]	@ 0x48
     ef6:	6c60      	ldr	r0, [r4, #68]	@ 0x44
     ef8:	0017      	movs	r7, r2
     efa:	3744      	adds	r7, #68	@ 0x44
     efc:	0045      	lsls	r5, r0, #1
     efe:	429f      	cmp	r7, r3
     f00:	da55      	bge.n	0xfae
     f02:	42b0      	cmp	r0, r6
     f04:	dd48      	ble.n	0xf98
     f06:	6465      	str	r5, [r4, #68]	@ 0x44
     f08:	2900      	cmp	r1, #0
     f0a:	d06c      	beq.n	0xfe6
     f0c:	6b63      	ldr	r3, [r4, #52]	@ 0x34
     f0e:	6b25      	ldr	r5, [r4, #48]	@ 0x30
     f10:	b2da      	uxtb	r2, r3
     f12:	6362      	str	r2, [r4, #52]	@ 0x34
     f14:	2d00      	cmp	r5, #0
     f16:	d108      	bne.n	0xf2a
     f18:	079b      	lsls	r3, r3, #30
     f1a:	d006      	beq.n	0xf2a
     f1c:	0022      	movs	r2, r4
     f1e:	2380      	movs	r3, #128	@ 0x80
     f20:	3258      	adds	r2, #88	@ 0x58
     f22:	8811      	ldrh	r1, [r2, #0]
     f24:	005b      	lsls	r3, r3, #1
     f26:	430b      	orrs	r3, r1
     f28:	8013      	strh	r3, [r2, #0]
     f2a:	68a0      	ldr	r0, [r4, #8]
     f2c:	6803      	ldr	r3, [r0, #0]
     f2e:	6b5b      	ldr	r3, [r3, #52]	@ 0x34
     f30:	4798      	blx	r3
     f32:	4285      	cmp	r5, r0
     f34:	da50      	bge.n	0xfd8
     f36:	6b23      	ldr	r3, [r4, #48]	@ 0x30
     f38:	6c22      	ldr	r2, [r4, #64]	@ 0x40
     f3a:	6b61      	ldr	r1, [r4, #52]	@ 0x34
     f3c:	1c58      	adds	r0, r3, #1
     f3e:	6320      	str	r0, [r4, #48]	@ 0x30
     f40:	54d1      	strb	r1, [r2, r3]
     f42:	6d63      	ldr	r3, [r4, #84]	@ 0x54
     f44:	6b62      	ldr	r2, [r4, #52]	@ 0x34
     f46:	4053      	eors	r3, r2
     f48:	6563      	str	r3, [r4, #84]	@ 0x54
     f4a:	6ce3      	ldr	r3, [r4, #76]	@ 0x4c
     f4c:	2b00      	cmp	r3, #0
     f4e:	d105      	bne.n	0xf5c
     f50:	0021      	movs	r1, r4
     f52:	2208      	movs	r2, #8
     f54:	3158      	adds	r1, #88	@ 0x58
     f56:	8808      	ldrh	r0, [r1, #0]
     f58:	4302      	orrs	r2, r0
     f5a:	800a      	strh	r2, [r1, #0]
     f5c:	6d22      	ldr	r2, [r4, #80]	@ 0x50
     f5e:	7ea1      	ldrb	r1, [r4, #26]
     f60:	4013      	ands	r3, r2
     f62:	6523      	str	r3, [r4, #80]	@ 0x50
     f64:	2303      	movs	r3, #3
     f66:	76e3      	strb	r3, [r4, #27]
     f68:	68e3      	ldr	r3, [r4, #12]
     f6a:	008a      	lsls	r2, r1, #2
     f6c:	681d      	ldr	r5, [r3, #0]
     f6e:	002b      	movs	r3, r5
     f70:	3318      	adds	r3, #24
     f72:	189b      	adds	r3, r3, r2
     f74:	22ed      	movs	r2, #237	@ 0xed
     f76:	0028      	movs	r0, r5
     f78:	601a      	str	r2, [r3, #0]
     f7a:	3aec      	subs	r2, #236	@ 0xec
     f7c:	f7ff fba4 	bl	0x6c8
     f80:	2281      	movs	r2, #129	@ 0x81
     f82:	0028      	movs	r0, r5
     f84:	7e21      	ldrb	r1, [r4, #24]
     f86:	f7ff fb8f 	bl	0x6a8
     f8a:	e628      	b.n	0xbde
     f8c:	7e22      	ldrb	r2, [r4, #24]
     f8e:	332c      	adds	r3, #44	@ 0x2c
     f90:	0092      	lsls	r2, r2, #2
     f92:	189b      	adds	r3, r3, r2
     f94:	681b      	ldr	r3, [r3, #0]
     f96:	e7a4      	b.n	0xee2
     f98:	6b67      	ldr	r7, [r4, #52]	@ 0x34
     f9a:	3268      	adds	r2, #104	@ 0x68
     f9c:	4338      	orrs	r0, r7
     f9e:	6360      	str	r0, [r4, #52]	@ 0x34
     fa0:	6ce0      	ldr	r0, [r4, #76]	@ 0x4c
     fa2:	64a2      	str	r2, [r4, #72]	@ 0x48
     fa4:	4247      	negs	r7, r0
     fa6:	4178      	adcs	r0, r7
     fa8:	6465      	str	r5, [r4, #68]	@ 0x44
     faa:	64e0      	str	r0, [r4, #76]	@ 0x4c
     fac:	e7a2      	b.n	0xef4
     fae:	3221      	adds	r2, #33	@ 0x21
     fb0:	429a      	cmp	r2, r3
     fb2:	daa8      	bge.n	0xf06
     fb4:	2280      	movs	r2, #128	@ 0x80
     fb6:	0052      	lsls	r2, r2, #1
     fb8:	4290      	cmp	r0, r2
     fba:	dca4      	bgt.n	0xf06
     fbc:	0020      	movs	r0, r4
     fbe:	3058      	adds	r0, #88	@ 0x58
     fc0:	8806      	ldrh	r6, [r0, #0]
     fc2:	3afc      	subs	r2, #252	@ 0xfc
     fc4:	4332      	orrs	r2, r6
     fc6:	8002      	strh	r2, [r0, #0]
     fc8:	e79d      	b.n	0xf06
     fca:	0020      	movs	r0, r4
     fcc:	2204      	movs	r2, #4
     fce:	3058      	adds	r0, #88	@ 0x58
     fd0:	8805      	ldrh	r5, [r0, #0]
     fd2:	432a      	orrs	r2, r5
     fd4:	8002      	strh	r2, [r0, #0]
     fd6:	e797      	b.n	0xf08
     fd8:	0022      	movs	r2, r4
     fda:	2320      	movs	r3, #32
     fdc:	3258      	adds	r2, #88	@ 0x58
     fde:	8811      	ldrh	r1, [r2, #0]
     fe0:	430b      	orrs	r3, r1
     fe2:	8013      	strh	r3, [r2, #0]
     fe4:	e7b1      	b.n	0xf4a
     fe6:	2282      	movs	r2, #130	@ 0x82
     fe8:	00d2      	lsls	r2, r2, #3
     fea:	4293      	cmp	r3, r2
     fec:	dc00      	bgt.n	0xff0
     fee:	e5f6      	b.n	0xbde
     ff0:	0022      	movs	r2, r4
     ff2:	2301      	movs	r3, #1
     ff4:	3258      	adds	r2, #88	@ 0x58
     ff6:	8811      	ldrh	r1, [r2, #0]
     ff8:	430b      	orrs	r3, r1
     ffa:	8013      	strh	r3, [r2, #0]
     ffc:	e5ef      	b.n	0xbde
     ffe:	9b00      	ldr	r3, [sp, #0]
    1000:	2b00      	cmp	r3, #0
    1002:	d004      	beq.n	0x100e
    1004:	2300      	movs	r3, #0
    1006:	62e3      	str	r3, [r4, #44]	@ 0x2c
    1008:	2302      	movs	r3, #2
    100a:	76e3      	strb	r3, [r4, #27]
    100c:	e5c5      	b.n	0xb9a
    100e:	9b00      	ldr	r3, [sp, #0]
    1010:	2168      	movs	r1, #104	@ 0x68
    1012:	63a3      	str	r3, [r4, #56]	@ 0x38
    1014:	68e3      	ldr	r3, [r4, #12]
    1016:	681d      	ldr	r5, [r3, #0]
    1018:	7e63      	ldrb	r3, [r4, #25]
    101a:	002a      	movs	r2, r5
    101c:	009b      	lsls	r3, r3, #2
    101e:	3218      	adds	r2, #24
    1020:	18d3      	adds	r3, r2, r3
    1022:	6019      	str	r1, [r3, #0]
    1024:	7ea1      	ldrb	r1, [r4, #26]
    1026:	0028      	movs	r0, r5
    1028:	008b      	lsls	r3, r1, #2
    102a:	18d3      	adds	r3, r2, r3
    102c:	228a      	movs	r2, #138	@ 0x8a
    102e:	601a      	str	r2, [r3, #0]
    1030:	3a87      	subs	r2, #135	@ 0x87
    1032:	f7ff fb49 	bl	0x6c8
    1036:	2281      	movs	r2, #129	@ 0x81
    1038:	0028      	movs	r0, r5
    103a:	7e21      	ldrb	r1, [r4, #24]
    103c:	f7ff fb34 	bl	0x6a8
    1040:	9b00      	ldr	r3, [sp, #0]
    1042:	9a00      	ldr	r2, [sp, #0]
    1044:	6323      	str	r3, [r4, #48]	@ 0x30
    1046:	0023      	movs	r3, r4
    1048:	335a      	adds	r3, #90	@ 0x5a
    104a:	801a      	strh	r2, [r3, #0]
    104c:	2307      	movs	r3, #7
    104e:	76e3      	strb	r3, [r4, #27]
    1050:	e5c5      	b.n	0xbde
    1052:	9b00      	ldr	r3, [sp, #0]
    1054:	2b00      	cmp	r3, #0
    1056:	d1d7      	bne.n	0x1008
    1058:	0026      	movs	r6, r4
    105a:	365e      	adds	r6, #94	@ 0x5e
    105c:	7833      	ldrb	r3, [r6, #0]
    105e:	2b00      	cmp	r3, #0
    1060:	d007      	beq.n	0x1072
    1062:	69e2      	ldr	r2, [r4, #28]
    1064:	6a23      	ldr	r3, [r4, #32]
    1066:	429a      	cmp	r2, r3
    1068:	da08      	bge.n	0x107c
    106a:	6a62      	ldr	r2, [r4, #36]	@ 0x24
    106c:	6aa3      	ldr	r3, [r4, #40]	@ 0x28
    106e:	429a      	cmp	r2, r3
    1070:	da04      	bge.n	0x107c
    1072:	0023      	movs	r3, r4
    1074:	335f      	adds	r3, #95	@ 0x5f
    1076:	781b      	ldrb	r3, [r3, #0]
    1078:	2b03      	cmp	r3, #3
    107a:	d908      	bls.n	0x108e
    107c:	0022      	movs	r2, r4
    107e:	2380      	movs	r3, #128	@ 0x80
    1080:	325a      	adds	r2, #90	@ 0x5a
    1082:	8811      	ldrh	r1, [r2, #0]
    1084:	0020      	movs	r0, r4
    1086:	430b      	orrs	r3, r1
    1088:	8013      	strh	r3, [r2, #0]
    108a:	f7ff f9b8 	bl	0x3fe
    108e:	6be1      	ldr	r1, [r4, #60]	@ 0x3c
    1090:	2900      	cmp	r1, #0
    1092:	d060      	beq.n	0x1156
    1094:	220f      	movs	r2, #15
    1096:	794b      	ldrb	r3, [r1, #5]
    1098:	4013      	ands	r3, r2
    109a:	3308      	adds	r3, #8
    109c:	63a3      	str	r3, [r4, #56]	@ 0x38
    109e:	7833      	ldrb	r3, [r6, #0]
    10a0:	2b00      	cmp	r3, #0
    10a2:	d00b      	beq.n	0x10bc
    10a4:	2320      	movs	r3, #32
    10a6:	780a      	ldrb	r2, [r1, #0]
    10a8:	421a      	tst	r2, r3
    10aa:	d007      	beq.n	0x10bc
    10ac:	439a      	bics	r2, r3
    10ae:	700a      	strb	r2, [r1, #0]
    10b0:	6ba2      	ldr	r2, [r4, #56]	@ 0x38
    10b2:	6be1      	ldr	r1, [r4, #60]	@ 0x3c
    10b4:	3a01      	subs	r2, #1
    10b6:	5c88      	ldrb	r0, [r1, r2]
    10b8:	4043      	eors	r3, r0
    10ba:	548b      	strb	r3, [r1, r2]
    10bc:	6be3      	ldr	r3, [r4, #60]	@ 0x3c
    10be:	6a21      	ldr	r1, [r4, #32]
    10c0:	781d      	ldrb	r5, [r3, #0]
    10c2:	2324      	movs	r3, #36	@ 0x24
    10c4:	401d      	ands	r5, r3
    10c6:	3d24      	subs	r5, #36	@ 0x24
    10c8:	1e6b      	subs	r3, r5, #1
    10ca:	419d      	sbcs	r5, r3
    10cc:	4b45      	ldr	r3, [pc, #276]	@ (0x11e4)
    10ce:	426d      	negs	r5, r5
    10d0:	401d      	ands	r5, r3
    10d2:	35a2      	adds	r5, #162	@ 0xa2
    10d4:	7833      	ldrb	r3, [r6, #0]
    10d6:	35ff      	adds	r5, #255	@ 0xff
    10d8:	2900      	cmp	r1, #0
    10da:	dc17      	bgt.n	0x110c
    10dc:	2b00      	cmp	r3, #0
    10de:	d135      	bne.n	0x114c
    10e0:	68e3      	ldr	r3, [r4, #12]
    10e2:	6818      	ldr	r0, [r3, #0]
    10e4:	7e63      	ldrb	r3, [r4, #25]
    10e6:	0002      	movs	r2, r0
    10e8:	009b      	lsls	r3, r3, #2
    10ea:	3218      	adds	r2, #24
    10ec:	18d3      	adds	r3, r2, r3
    10ee:	601d      	str	r5, [r3, #0]
    10f0:	7ea1      	ldrb	r1, [r4, #26]
    10f2:	3522      	adds	r5, #34	@ 0x22
    10f4:	008b      	lsls	r3, r1, #2
    10f6:	18d3      	adds	r3, r2, r3
    10f8:	601d      	str	r5, [r3, #0]
    10fa:	2203      	movs	r2, #3
    10fc:	f7ff fae4 	bl	0x6c8
    1100:	0022      	movs	r2, r4
    1102:	2300      	movs	r3, #0
    1104:	325a      	adds	r2, #90	@ 0x5a
    1106:	6323      	str	r3, [r4, #48]	@ 0x30
    1108:	8013      	strh	r3, [r2, #0]
    110a:	e79f      	b.n	0x104c
    110c:	6aa2      	ldr	r2, [r4, #40]	@ 0x28
    110e:	2b00      	cmp	r3, #0
    1110:	d01e      	beq.n	0x1150
    1112:	17d3      	asrs	r3, r2, #31
    1114:	1a9b      	subs	r3, r3, r2
    1116:	0fdb      	lsrs	r3, r3, #31
    1118:	69e2      	ldr	r2, [r4, #28]
    111a:	3201      	adds	r2, #1
    111c:	4291      	cmp	r1, r2
    111e:	dddf      	ble.n	0x10e0
    1120:	6a62      	ldr	r2, [r4, #36]	@ 0x24
    1122:	6aa1      	ldr	r1, [r4, #40]	@ 0x28
    1124:	3201      	adds	r2, #1
    1126:	428a      	cmp	r2, r1
    1128:	dada      	bge.n	0x10e0
    112a:	2b00      	cmp	r3, #0
    112c:	d0d8      	beq.n	0x10e0
    112e:	0023      	movs	r3, r4
    1130:	335f      	adds	r3, #95	@ 0x5f
    1132:	781b      	ldrb	r3, [r3, #0]
    1134:	2b03      	cmp	r3, #3
    1136:	d0d3      	beq.n	0x10e0
    1138:	2083      	movs	r0, #131	@ 0x83
    113a:	213a      	movs	r1, #58	@ 0x3a
    113c:	4b2a      	ldr	r3, [pc, #168]	@ (0x11e8)
    113e:	31ff      	adds	r1, #255	@ 0xff
    1140:	681b      	ldr	r3, [r3, #0]
    1142:	4358      	muls	r0, r3
    1144:	f001 fd9e 	bl	0x2c84
    1148:	194d      	adds	r5, r1, r5
    114a:	e7c9      	b.n	0x10e0
    114c:	2300      	movs	r3, #0
    114e:	e7e3      	b.n	0x1118
    1150:	2a00      	cmp	r2, #0
    1152:	dcec      	bgt.n	0x112e
    1154:	e7c4      	b.n	0x10e0
    1156:	68e3      	ldr	r3, [r4, #12]
    1158:	2281      	movs	r2, #129	@ 0x81
    115a:	681d      	ldr	r5, [r3, #0]
    115c:	7e21      	ldrb	r1, [r4, #24]
    115e:	0028      	movs	r0, r5
    1160:	f7ff faa2 	bl	0x6a8
    1164:	7ea6      	ldrb	r6, [r4, #26]
    1166:	0028      	movs	r0, r5
    1168:	0031      	movs	r1, r6
    116a:	2202      	movs	r2, #2
    116c:	f7ff faac 	bl	0x6c8
    1170:	3518      	adds	r5, #24
    1172:	4b1e      	ldr	r3, [pc, #120]	@ (0x11ec)
    1174:	00b6      	lsls	r6, r6, #2
    1176:	19ae      	adds	r6, r5, r6
    1178:	6033      	str	r3, [r6, #0]
    117a:	7e63      	ldrb	r3, [r4, #25]
    117c:	009b      	lsls	r3, r3, #2
    117e:	18ed      	adds	r5, r5, r3
    1180:	4b1b      	ldr	r3, [pc, #108]	@ (0x11f0)
    1182:	602b      	str	r3, [r5, #0]
    1184:	2301      	movs	r3, #1
    1186:	e762      	b.n	0x104e
    1188:	68e3      	ldr	r3, [r4, #12]
    118a:	7ea1      	ldrb	r1, [r4, #26]
    118c:	6818      	ldr	r0, [r3, #0]
    118e:	2301      	movs	r3, #1
    1190:	408b      	lsls	r3, r1
    1192:	6802      	ldr	r2, [r0, #0]
    1194:	421a      	tst	r2, r3
    1196:	d12d      	bne.n	0x11f4
    1198:	0003      	movs	r3, r0
    119a:	7e22      	ldrb	r2, [r4, #24]
    119c:	332c      	adds	r3, #44	@ 0x2c
    119e:	0092      	lsls	r2, r2, #2
    11a0:	189b      	adds	r3, r3, r2
    11a2:	0002      	movs	r2, r0
    11a4:	7e65      	ldrb	r5, [r4, #25]
    11a6:	3218      	adds	r2, #24
    11a8:	00ad      	lsls	r5, r5, #2
    11aa:	1955      	adds	r5, r2, r5
    11ac:	681b      	ldr	r3, [r3, #0]
    11ae:	682e      	ldr	r6, [r5, #0]
    11b0:	0037      	movs	r7, r6
    11b2:	3f1e      	subs	r7, #30
    11b4:	429f      	cmp	r7, r3
    11b6:	d908      	bls.n	0x11ca
    11b8:	6b23      	ldr	r3, [r4, #48]	@ 0x30
    11ba:	2b00      	cmp	r3, #0
    11bc:	d002      	beq.n	0x11c4
    11be:	0020      	movs	r0, r4
    11c0:	f7ff f8ec 	bl	0x39c
    11c4:	4b0a      	ldr	r3, [pc, #40]	@ (0x11f0)
    11c6:	602b      	str	r3, [r5, #0]
    11c8:	e71e      	b.n	0x1008
    11ca:	429e      	cmp	r6, r3
    11cc:	d906      	bls.n	0x11dc
    11ce:	6880      	ldr	r0, [r0, #8]
    11d0:	0089      	lsls	r1, r1, #2
    11d2:	3001      	adds	r0, #1
    11d4:	3322      	adds	r3, #34	@ 0x22
    11d6:	1852      	adds	r2, r2, r1
    11d8:	6028      	str	r0, [r5, #0]
    11da:	6013      	str	r3, [r2, #0]
    11dc:	2308      	movs	r3, #8
    11de:	e736      	b.n	0x104e
    11e0:	0477      	lsls	r7, r6, #17
    11e2:	0000      	movs	r0, r0
    11e4:	fec7 ffff 	mcr2	15, 6, pc, cr7, cr15, {7}	@ <UNPREDICTABLE>
    11e8:	0070      	lsls	r0, r6, #1
    11ea:	1000      	asrs	r0, r0, #32
    11ec:	fffe 0000 	vaddl.u<illegal width 64>	q8, d14, d0
    11f0:	ffff 0000 	vaddl.u<illegal width 64>	q8, d15, d0
    11f4:	2308      	movs	r3, #8
    11f6:	0022      	movs	r2, r4
    11f8:	76e3      	strb	r3, [r4, #27]
    11fa:	325a      	adds	r2, #90	@ 0x5a
    11fc:	8811      	ldrh	r1, [r2, #0]
    11fe:	3318      	adds	r3, #24
    1200:	430b      	orrs	r3, r1
    1202:	8013      	strh	r3, [r2, #0]
    1204:	6ae3      	ldr	r3, [r4, #44]	@ 0x2c
    1206:	2b00      	cmp	r3, #0
    1208:	d104      	bne.n	0x1214
    120a:	6b23      	ldr	r3, [r4, #48]	@ 0x30
    120c:	6be2      	ldr	r2, [r4, #60]	@ 0x3c
    120e:	1c59      	adds	r1, r3, #1
    1210:	6321      	str	r1, [r4, #48]	@ 0x30
    1212:	5cd3      	ldrb	r3, [r2, r3]
    1214:	2180      	movs	r1, #128	@ 0x80
    1216:	6363      	str	r3, [r4, #52]	@ 0x34
    1218:	2301      	movs	r3, #1
    121a:	0049      	lsls	r1, r1, #1
    121c:	6b62      	ldr	r2, [r4, #52]	@ 0x34
    121e:	4213      	tst	r3, r2
    1220:	d001      	beq.n	0x1226
    1222:	404a      	eors	r2, r1
    1224:	6362      	str	r2, [r4, #52]	@ 0x34
    1226:	005b      	lsls	r3, r3, #1
    1228:	2bff      	cmp	r3, #255	@ 0xff
    122a:	ddf7      	ble.n	0x121c
    122c:	2301      	movs	r3, #1
    122e:	6463      	str	r3, [r4, #68]	@ 0x44
    1230:	3308      	adds	r3, #8
    1232:	76e3      	strb	r3, [r4, #27]
    1234:	2601      	movs	r6, #1
    1236:	68e3      	ldr	r3, [r4, #12]
    1238:	7ea1      	ldrb	r1, [r4, #26]
    123a:	681d      	ldr	r5, [r3, #0]
    123c:	0033      	movs	r3, r6
    123e:	682a      	ldr	r2, [r5, #0]
    1240:	408b      	lsls	r3, r1
    1242:	0010      	movs	r0, r2
    1244:	4018      	ands	r0, r3
    1246:	9001      	str	r0, [sp, #4]
    1248:	421a      	tst	r2, r3
    124a:	d178      	bne.n	0x133e
    124c:	002b      	movs	r3, r5
    124e:	7e22      	ldrb	r2, [r4, #24]
    1250:	332c      	adds	r3, #44	@ 0x2c
    1252:	0092      	lsls	r2, r2, #2
    1254:	189b      	adds	r3, r3, r2
    1256:	681f      	ldr	r7, [r3, #0]
    1258:	2f02      	cmp	r7, #2
    125a:	d800      	bhi.n	0x125e
    125c:	e4bf      	b.n	0xbde
    125e:	2168      	movs	r1, #104	@ 0x68
    1260:	0038      	movs	r0, r7
    1262:	f001 fd0f 	bl	0x2c84
    1266:	293d      	cmp	r1, #61	@ 0x3d
    1268:	d806      	bhi.n	0x1278
    126a:	0020      	movs	r0, r4
    126c:	f7ff f896 	bl	0x39c
    1270:	0020      	movs	r0, r4
    1272:	f7ff faad 	bl	0x7d0
    1276:	e4b2      	b.n	0xbde
    1278:	7e63      	ldrb	r3, [r4, #25]
    127a:	3518      	adds	r5, #24
    127c:	009b      	lsls	r3, r3, #2
    127e:	18ed      	adds	r5, r5, r3
    1280:	682b      	ldr	r3, [r5, #0]
    1282:	3b07      	subs	r3, #7
    1284:	42bb      	cmp	r3, r7
    1286:	d800      	bhi.n	0x128a
    1288:	e4a9      	b.n	0xbde
    128a:	0020      	movs	r0, r4
    128c:	f7ff f886 	bl	0x39c
    1290:	0023      	movs	r3, r4
    1292:	9a01      	ldr	r2, [sp, #4]
    1294:	3358      	adds	r3, #88	@ 0x58
    1296:	801a      	strh	r2, [r3, #0]
    1298:	23ff      	movs	r3, #255	@ 0xff
    129a:	6563      	str	r3, [r4, #84]	@ 0x54
    129c:	6ae3      	ldr	r3, [r4, #44]	@ 0x2c
    129e:	6526      	str	r6, [r4, #80]	@ 0x50
    12a0:	64e6      	str	r6, [r4, #76]	@ 0x4c
    12a2:	2b00      	cmp	r3, #0
    12a4:	d031      	beq.n	0x130a
    12a6:	62e2      	str	r2, [r4, #44]	@ 0x2c
    12a8:	7ea3      	ldrb	r3, [r4, #26]
    12aa:	2168      	movs	r1, #104	@ 0x68
    12ac:	9301      	str	r3, [sp, #4]
    12ae:	68e3      	ldr	r3, [r4, #12]
    12b0:	681b      	ldr	r3, [r3, #0]
    12b2:	001e      	movs	r6, r3
    12b4:	9303      	str	r3, [sp, #12]
    12b6:	9b01      	ldr	r3, [sp, #4]
    12b8:	3618      	adds	r6, #24
    12ba:	009d      	lsls	r5, r3, #2
    12bc:	1975      	adds	r5, r6, r5
    12be:	6828      	ldr	r0, [r5, #0]
    12c0:	3802      	subs	r0, #2
    12c2:	1bc0      	subs	r0, r0, r7
    12c4:	f001 fc58 	bl	0x2b78
    12c8:	6c63      	ldr	r3, [r4, #68]	@ 0x44
    12ca:	003a      	movs	r2, r7
    12cc:	3001      	adds	r0, #1
    12ce:	4103      	asrs	r3, r0
    12d0:	3a68      	subs	r2, #104	@ 0x68
    12d2:	64a2      	str	r2, [r4, #72]	@ 0x48
    12d4:	6b62      	ldr	r2, [r4, #52]	@ 0x34
    12d6:	1e59      	subs	r1, r3, #1
    12d8:	400a      	ands	r2, r1
    12da:	210a      	movs	r1, #10
    12dc:	6463      	str	r3, [r4, #68]	@ 0x44
    12de:	9202      	str	r2, [sp, #8]
    12e0:	6362      	str	r2, [r4, #52]	@ 0x34
    12e2:	105b      	asrs	r3, r3, #1
    12e4:	2b00      	cmp	r3, #0
    12e6:	d11f      	bne.n	0x1328
    12e8:	3368      	adds	r3, #104	@ 0x68
    12ea:	4359      	muls	r1, r3
    12ec:	1e7b      	subs	r3, r7, #1
    12ee:	18cb      	adds	r3, r1, r3
    12f0:	2203      	movs	r2, #3
    12f2:	602b      	str	r3, [r5, #0]
    12f4:	9901      	ldr	r1, [sp, #4]
    12f6:	9803      	ldr	r0, [sp, #12]
    12f8:	f7ff f9e6 	bl	0x6c8
    12fc:	7e63      	ldrb	r3, [r4, #25]
    12fe:	4a66      	ldr	r2, [pc, #408]	@ (0x1498)
    1300:	009b      	lsls	r3, r3, #2
    1302:	18f3      	adds	r3, r6, r3
    1304:	601a      	str	r2, [r3, #0]
    1306:	2304      	movs	r3, #4
    1308:	e67f      	b.n	0x100a
    130a:	6b22      	ldr	r2, [r4, #48]	@ 0x30
    130c:	3a01      	subs	r2, #1
    130e:	6322      	str	r2, [r4, #48]	@ 0x30
    1310:	6b22      	ldr	r2, [r4, #48]	@ 0x30
    1312:	429a      	cmp	r2, r3
    1314:	ddc8      	ble.n	0x12a8
    1316:	6be2      	ldr	r2, [r4, #60]	@ 0x3c
    1318:	5cd1      	ldrb	r1, [r2, r3]
    131a:	6c22      	ldr	r2, [r4, #64]	@ 0x40
    131c:	54d1      	strb	r1, [r2, r3]
    131e:	6d62      	ldr	r2, [r4, #84]	@ 0x54
    1320:	3301      	adds	r3, #1
    1322:	404a      	eors	r2, r1
    1324:	6562      	str	r2, [r4, #84]	@ 0x54
    1326:	e7f3      	b.n	0x1310
    1328:	9a02      	ldr	r2, [sp, #8]
    132a:	3901      	subs	r1, #1
    132c:	4213      	tst	r3, r2
    132e:	d004      	beq.n	0x133a
    1330:	6ce0      	ldr	r0, [r4, #76]	@ 0x4c
    1332:	0002      	movs	r2, r0
    1334:	4250      	negs	r0, r2
    1336:	4150      	adcs	r0, r2
    1338:	64e0      	str	r0, [r4, #76]	@ 0x4c
    133a:	105b      	asrs	r3, r3, #1
    133c:	e7d2      	b.n	0x12e4
    133e:	2380      	movs	r3, #128	@ 0x80
    1340:	6c62      	ldr	r2, [r4, #68]	@ 0x44
    1342:	009b      	lsls	r3, r3, #2
    1344:	429a      	cmp	r2, r3
    1346:	dc1f      	bgt.n	0x1388
    1348:	2680      	movs	r6, #128	@ 0x80
    134a:	3b99      	subs	r3, #153	@ 0x99
    134c:	6b67      	ldr	r7, [r4, #52]	@ 0x34
    134e:	3bff      	subs	r3, #255	@ 0xff
    1350:	0076      	lsls	r6, r6, #1
    1352:	6c62      	ldr	r2, [r4, #68]	@ 0x44
    1354:	0050      	lsls	r0, r2, #1
    1356:	6460      	str	r0, [r4, #68]	@ 0x44
    1358:	423a      	tst	r2, r7
    135a:	d001      	beq.n	0x1360
    135c:	42b2      	cmp	r2, r6
    135e:	dd0e      	ble.n	0x137e
    1360:	2680      	movs	r6, #128	@ 0x80
    1362:	7e62      	ldrb	r2, [r4, #25]
    1364:	3518      	adds	r5, #24
    1366:	0092      	lsls	r2, r2, #2
    1368:	18aa      	adds	r2, r5, r2
    136a:	00b6      	lsls	r6, r6, #2
    136c:	42b0      	cmp	r0, r6
    136e:	dd08      	ble.n	0x1382
    1370:	4849      	ldr	r0, [pc, #292]	@ (0x1498)
    1372:	0089      	lsls	r1, r1, #2
    1374:	6010      	str	r0, [r2, #0]
    1376:	3b01      	subs	r3, #1
    1378:	186d      	adds	r5, r5, r1
    137a:	602b      	str	r3, [r5, #0]
    137c:	e42f      	b.n	0xbde
    137e:	3368      	adds	r3, #104	@ 0x68
    1380:	e7e7      	b.n	0x1352
    1382:	0018      	movs	r0, r3
    1384:	3823      	subs	r0, #35	@ 0x23
    1386:	e7f4      	b.n	0x1372
    1388:	68e3      	ldr	r3, [r4, #12]
    138a:	6b22      	ldr	r2, [r4, #48]	@ 0x30
    138c:	6818      	ldr	r0, [r3, #0]
    138e:	6ba3      	ldr	r3, [r4, #56]	@ 0x38
    1390:	0005      	movs	r5, r0
    1392:	3518      	adds	r5, #24
    1394:	429a      	cmp	r2, r3
    1396:	da11      	bge.n	0x13bc
    1398:	6ae3      	ldr	r3, [r4, #44]	@ 0x2c
    139a:	2b00      	cmp	r3, #0
    139c:	d10e      	bne.n	0x13bc
    139e:	228b      	movs	r2, #139	@ 0x8b
    13a0:	3307      	adds	r3, #7
    13a2:	76e3      	strb	r3, [r4, #27]
    13a4:	7e63      	ldrb	r3, [r4, #25]
    13a6:	0052      	lsls	r2, r2, #1
    13a8:	009b      	lsls	r3, r3, #2
    13aa:	18eb      	adds	r3, r5, r3
    13ac:	601a      	str	r2, [r3, #0]
    13ae:	233a      	movs	r3, #58	@ 0x3a
    13b0:	33ff      	adds	r3, #255	@ 0xff
    13b2:	7ea2      	ldrb	r2, [r4, #26]
    13b4:	3b01      	subs	r3, #1
    13b6:	0092      	lsls	r2, r2, #2
    13b8:	18ad      	adds	r5, r5, r2
    13ba:	e7de      	b.n	0x137a
    13bc:	230b      	movs	r3, #11
    13be:	2280      	movs	r2, #128	@ 0x80
    13c0:	76e3      	strb	r3, [r4, #27]
    13c2:	7e21      	ldrb	r1, [r4, #24]
    13c4:	f7ff f970 	bl	0x6a8
    13c8:	2345      	movs	r3, #69	@ 0x45
    13ca:	e7f2      	b.n	0x13b2
    13cc:	68e3      	ldr	r3, [r4, #12]
    13ce:	681d      	ldr	r5, [r3, #0]
    13d0:	6ae3      	ldr	r3, [r4, #44]	@ 0x2c
    13d2:	2b00      	cmp	r3, #0
    13d4:	d011      	beq.n	0x13fa
    13d6:	2300      	movs	r3, #0
    13d8:	62e3      	str	r3, [r4, #44]	@ 0x2c
    13da:	3306      	adds	r3, #6
    13dc:	76e3      	strb	r3, [r4, #27]
    13de:	2281      	movs	r2, #129	@ 0x81
    13e0:	0028      	movs	r0, r5
    13e2:	7e21      	ldrb	r1, [r4, #24]
    13e4:	f7ff f960 	bl	0x6a8
    13e8:	4b2c      	ldr	r3, [pc, #176]	@ (0x149c)
    13ea:	7ea2      	ldrb	r2, [r4, #26]
    13ec:	3518      	adds	r5, #24
    13ee:	0092      	lsls	r2, r2, #2
    13f0:	3b01      	subs	r3, #1
    13f2:	18a8      	adds	r0, r5, r2
    13f4:	6003      	str	r3, [r0, #0]
    13f6:	f7ff fbf2 	bl	0xbde
    13fa:	0023      	movs	r3, r4
    13fc:	2201      	movs	r2, #1
    13fe:	335c      	adds	r3, #92	@ 0x5c
    1400:	701a      	strb	r2, [r3, #0]
    1402:	230c      	movs	r3, #12
    1404:	0028      	movs	r0, r5
    1406:	76e3      	strb	r3, [r4, #27]
    1408:	7ea1      	ldrb	r1, [r4, #26]
    140a:	f7ff f95d 	bl	0x6c8
    140e:	0023      	movs	r3, r4
    1410:	335e      	adds	r3, #94	@ 0x5e
    1412:	781b      	ldrb	r3, [r3, #0]
    1414:	2b00      	cmp	r3, #0
    1416:	d007      	beq.n	0x1428
    1418:	0023      	movs	r3, r4
    141a:	335d      	adds	r3, #93	@ 0x5d
    141c:	781b      	ldrb	r3, [r3, #0]
    141e:	2b00      	cmp	r3, #0
    1420:	d004      	beq.n	0x142c
    1422:	6a63      	ldr	r3, [r4, #36]	@ 0x24
    1424:	3301      	adds	r3, #1
    1426:	6263      	str	r3, [r4, #36]	@ 0x24
    1428:	4b1d      	ldr	r3, [pc, #116]	@ (0x14a0)
    142a:	e7de      	b.n	0x13ea
    142c:	69e3      	ldr	r3, [r4, #28]
    142e:	3301      	adds	r3, #1
    1430:	61e3      	str	r3, [r4, #28]
    1432:	e7f9      	b.n	0x1428
    1434:	230d      	movs	r3, #13
    1436:	76e3      	strb	r3, [r4, #27]
    1438:	68e3      	ldr	r3, [r4, #12]
    143a:	2281      	movs	r2, #129	@ 0x81
    143c:	681d      	ldr	r5, [r3, #0]
    143e:	7e21      	ldrb	r1, [r4, #24]
    1440:	0028      	movs	r0, r5
    1442:	f7ff f931 	bl	0x6a8
    1446:	7ea3      	ldrb	r3, [r4, #26]
    1448:	3518      	adds	r5, #24
    144a:	009b      	lsls	r3, r3, #2
    144c:	18ed      	adds	r5, r5, r3
    144e:	4b15      	ldr	r3, [pc, #84]	@ (0x14a4)
    1450:	e793      	b.n	0x137a
    1452:	9b00      	ldr	r3, [sp, #0]
    1454:	2b00      	cmp	r3, #0
    1456:	d000      	beq.n	0x145a
    1458:	e5d6      	b.n	0x1008
    145a:	0023      	movs	r3, r4
    145c:	2201      	movs	r2, #1
    145e:	335e      	adds	r3, #94	@ 0x5e
    1460:	701a      	strb	r2, [r3, #0]
    1462:	9a00      	ldr	r2, [sp, #0]
    1464:	3b02      	subs	r3, #2
    1466:	701a      	strb	r2, [r3, #0]
    1468:	0022      	movs	r2, r4
    146a:	2308      	movs	r3, #8
    146c:	325a      	adds	r2, #90	@ 0x5a
    146e:	8811      	ldrh	r1, [r2, #0]
    1470:	430b      	orrs	r3, r1
    1472:	8013      	strh	r3, [r2, #0]
    1474:	2306      	movs	r3, #6
    1476:	76e3      	strb	r3, [r4, #27]
    1478:	68e3      	ldr	r3, [r4, #12]
    147a:	7ea1      	ldrb	r1, [r4, #26]
    147c:	6818      	ldr	r0, [r3, #0]
    147e:	008a      	lsls	r2, r1, #2
    1480:	0003      	movs	r3, r0
    1482:	3318      	adds	r3, #24
    1484:	189b      	adds	r3, r3, r2
    1486:	4a08      	ldr	r2, [pc, #32]	@ (0x14a8)
    1488:	601a      	str	r2, [r3, #0]
    148a:	2203      	movs	r2, #3
    148c:	f7ff f91c 	bl	0x6c8
    1490:	f7ff fba5 	bl	0xbde
    1494:	f7ff f9da 	bl	0x84c
    1498:	ffff 0000 	vaddl.u<illegal width 64>	q8, d15, d0
    149c:	13f0      	asrs	r0, r6, #15
    149e:	0000      	movs	r0, r0
    14a0:	05b2      	lsls	r2, r6, #22
    14a2:	0000      	movs	r0, r0
    14a4:	069c      	lsls	r4, r3, #26
    14a6:	0000      	movs	r0, r0
    14a8:	13ef      	asrs	r7, r5, #15
    14aa:	0000      	movs	r0, r0
    14ac:	b510      	push	{r4, lr}
    14ae:	4b02      	ldr	r3, [pc, #8]	@ (0x14b8)
    14b0:	6898      	ldr	r0, [r3, #8]
    14b2:	f7ff fb3d 	bl	0xb30
    14b6:	bd10      	pop	{r4, pc}
    14b8:	0070      	lsls	r0, r6, #1
    14ba:	1000      	asrs	r0, r0, #32
    14bc:	b570      	push	{r4, r5, r6, lr}
    14be:	4c0d      	ldr	r4, [pc, #52]	@ (0x14f4)
    14c0:	0006      	movs	r6, r0
    14c2:	68a5      	ldr	r5, [r4, #8]
    14c4:	2001      	movs	r0, #1
    14c6:	b2ad      	uxth	r5, r5
    14c8:	2e02      	cmp	r6, #2
    14ca:	d906      	bls.n	0x14da
    14cc:	4b0a      	ldr	r3, [pc, #40]	@ (0x14f8)
    14ce:	490b      	ldr	r1, [pc, #44]	@ (0x14fc)
    14d0:	6818      	ldr	r0, [r3, #0]
    14d2:	f001 fb51 	bl	0x2b78
    14d6:	3e02      	subs	r6, #2
    14d8:	4370      	muls	r0, r6
    14da:	2800      	cmp	r0, #0
    14dc:	dc00      	bgt.n	0x14e0
    14de:	bd70      	pop	{r4, r5, r6, pc}
    14e0:	68a3      	ldr	r3, [r4, #8]
    14e2:	b29a      	uxth	r2, r3
    14e4:	1aab      	subs	r3, r5, r2
    14e6:	d501      	bpl.n	0x14ec
    14e8:	6861      	ldr	r1, [r4, #4]
    14ea:	185b      	adds	r3, r3, r1
    14ec:	0015      	movs	r5, r2
    14ee:	1ac0      	subs	r0, r0, r3
    14f0:	e7f3      	b.n	0x14da
    14f2:	46c0      	nop			@ (mov r8, r8)
    14f4:	e010      	b.n	0x1518
    14f6:	e000      	b.n	0x14fa
    14f8:	001c      	movs	r4, r3
    14fa:	1000      	asrs	r0, r0, #32
    14fc:	4240      	negs	r0, r0
    14fe:	000f      	movs	r7, r1
    1500:	4b14      	ldr	r3, [pc, #80]	@ (0x1554)
    1502:	b570      	push	{r4, r5, r6, lr}
    1504:	681a      	ldr	r2, [r3, #0]
    1506:	07d2      	lsls	r2, r2, #31
    1508:	d404      	bmi.n	0x1514
    150a:	681b      	ldr	r3, [r3, #0]
    150c:	079b      	lsls	r3, r3, #30
    150e:	d401      	bmi.n	0x1514
    1510:	f7ff f99c 	bl	0x84c
    1514:	4b10      	ldr	r3, [pc, #64]	@ (0x1558)
    1516:	685b      	ldr	r3, [r3, #4]
    1518:	05db      	lsls	r3, r3, #23
    151a:	d00d      	beq.n	0x1538
    151c:	25fa      	movs	r5, #250	@ 0xfa
    151e:	243e      	movs	r4, #62	@ 0x3e
    1520:	012d      	lsls	r5, r5, #4
    1522:	0028      	movs	r0, r5
    1524:	3c01      	subs	r4, #1
    1526:	f7ff ffc9 	bl	0x14bc
    152a:	2c00      	cmp	r4, #0
    152c:	d1f9      	bne.n	0x1522
    152e:	20fa      	movs	r0, #250	@ 0xfa
    1530:	00c0      	lsls	r0, r0, #3
    1532:	f7ff ffc3 	bl	0x14bc
    1536:	bd70      	pop	{r4, r5, r6, pc}
    1538:	23fa      	movs	r3, #250	@ 0xfa
    153a:	4a08      	ldr	r2, [pc, #32]	@ (0x155c)
    153c:	6811      	ldr	r1, [r2, #0]
    153e:	6810      	ldr	r0, [r2, #0]
    1540:	4288      	cmp	r0, r1
    1542:	d101      	bne.n	0x1548
    1544:	bf30      	wfi
    1546:	e7fa      	b.n	0x153e
    1548:	3b01      	subs	r3, #1
    154a:	6811      	ldr	r1, [r2, #0]
    154c:	2b00      	cmp	r3, #0
    154e:	d1f6      	bne.n	0x153e
    1550:	e7f1      	b.n	0x1536
    1552:	46c0      	nop			@ (mov r8, r8)
    1554:	e010      	b.n	0x1578
    1556:	e000      	b.n	0x155a
    1558:	ed00 e000 	stc	0, cr14, [r0, #-0]
    155c:	0070      	lsls	r0, r6, #1
    155e:	1000      	asrs	r0, r0, #32
    1560:	2080      	movs	r0, #128	@ 0x80
    1562:	b570      	push	{r4, r5, r6, lr}
    1564:	0440      	lsls	r0, r0, #17
    1566:	f7ff f959 	bl	0x81c
    156a:	f7ff fad1 	bl	0xb10
    156e:	211f      	movs	r1, #31
    1570:	2201      	movs	r2, #1
    1572:	4b0a      	ldr	r3, [pc, #40]	@ (0x159c)
    1574:	2480      	movs	r4, #128	@ 0x80
    1576:	681b      	ldr	r3, [r3, #0]
    1578:	4d09      	ldr	r5, [pc, #36]	@ (0x15a0)
    157a:	4019      	ands	r1, r3
    157c:	408a      	lsls	r2, r1
    157e:	065b      	lsls	r3, r3, #25
    1580:	4908      	ldr	r1, [pc, #32]	@ (0x15a4)
    1582:	0f9b      	lsrs	r3, r3, #30
    1584:	009b      	lsls	r3, r3, #2
    1586:	5859      	ldr	r1, [r3, r1]
    1588:	03a4      	lsls	r4, r4, #14
    158a:	0090      	lsls	r0, r2, #2
    158c:	68ab      	ldr	r3, [r5, #8]
    158e:	4023      	ands	r3, r4
    1590:	425e      	negs	r6, r3
    1592:	4173      	adcs	r3, r6
    1594:	425b      	negs	r3, r3
    1596:	4013      	ands	r3, r2
    1598:	5043      	str	r3, [r0, r1]
    159a:	e7f7      	b.n	0x158c
    159c:	0014      	movs	r4, r2
    159e:	1000      	asrs	r0, r0, #32
    15a0:	e010      	b.n	0x15c4
    15a2:	e000      	b.n	0x15a6
    15a4:	2ff8      	cmp	r7, #248	@ 0xf8
    15a6:	0000      	movs	r0, r0
    15a8:	b510      	push	{r4, lr}
    15aa:	0004      	movs	r4, r0
    15ac:	f001 fb84 	bl	0x2cb8
    15b0:	0020      	movs	r0, r4
    15b2:	bd10      	pop	{r4, pc}
    15b4:	b5f7      	push	{r0, r1, r2, r4, r5, r6, r7, lr}
    15b6:	0007      	movs	r7, r0
    15b8:	f7fe fefe 	bl	0x3b8
    15bc:	2300      	movs	r3, #0
    15be:	6abc      	ldr	r4, [r7, #40]	@ 0x28
    15c0:	493b      	ldr	r1, [pc, #236]	@ (0x16b0)
    15c2:	6063      	str	r3, [r4, #4]
    15c4:	63e3      	str	r3, [r4, #60]	@ 0x3c
    15c6:	61e3      	str	r3, [r4, #28]
    15c8:	6263      	str	r3, [r4, #36]	@ 0x24
    15ca:	63a3      	str	r3, [r4, #56]	@ 0x38
    15cc:	65a3      	str	r3, [r4, #88]	@ 0x58
    15ce:	65e3      	str	r3, [r4, #92]	@ 0x5c
    15d0:	68e3      	ldr	r3, [r4, #12]
    15d2:	4a38      	ldr	r2, [pc, #224]	@ (0x16b4)
    15d4:	428b      	cmp	r3, r1
    15d6:	d14b      	bne.n	0x1670
    15d8:	21c4      	movs	r1, #196	@ 0xc4
    15da:	25ff      	movs	r5, #255	@ 0xff
    15dc:	4836      	ldr	r0, [pc, #216]	@ (0x16b8)
    15de:	0089      	lsls	r1, r1, #2
    15e0:	5842      	ldr	r2, [r0, r1]
    15e2:	43aa      	bics	r2, r5
    15e4:	5042      	str	r2, [r0, r1]
    15e6:	2601      	movs	r6, #1
    15e8:	0035      	movs	r5, r6
    15ea:	791a      	ldrb	r2, [r3, #4]
    15ec:	4933      	ldr	r1, [pc, #204]	@ (0x16bc)
    15ee:	6fc8      	ldr	r0, [r1, #124]	@ 0x7c
    15f0:	9201      	str	r2, [sp, #4]
    15f2:	3207      	adds	r2, #7
    15f4:	4095      	lsls	r5, r2
    15f6:	002a      	movs	r2, r5
    15f8:	681d      	ldr	r5, [r3, #0]
    15fa:	2300      	movs	r3, #0
    15fc:	4302      	orrs	r2, r0
    15fe:	67ca      	str	r2, [r1, #124]	@ 0x7c
    1600:	0031      	movs	r1, r6
    1602:	63eb      	str	r3, [r5, #60]	@ 0x3c
    1604:	616b      	str	r3, [r5, #20]
    1606:	62ab      	str	r3, [r5, #40]	@ 0x28
    1608:	7e63      	ldrb	r3, [r4, #25]
    160a:	6f6a      	ldr	r2, [r5, #116]	@ 0x74
    160c:	4099      	lsls	r1, r3
    160e:	000b      	movs	r3, r1
    1610:	4313      	orrs	r3, r2
    1612:	676b      	str	r3, [r5, #116]	@ 0x74
    1614:	686b      	ldr	r3, [r5, #4]
    1616:	492a      	ldr	r1, [pc, #168]	@ (0x16c0)
    1618:	4333      	orrs	r3, r6
    161a:	606b      	str	r3, [r5, #4]
    161c:	4b29      	ldr	r3, [pc, #164]	@ (0x16c4)
    161e:	6818      	ldr	r0, [r3, #0]
    1620:	f001 faaa 	bl	0x2b78
    1624:	3801      	subs	r0, #1
    1626:	60e8      	str	r0, [r5, #12]
    1628:	0020      	movs	r0, r4
    162a:	f7ff f8d1 	bl	0x7d0
    162e:	4b26      	ldr	r3, [pc, #152]	@ (0x16c8)
    1630:	60ab      	str	r3, [r5, #8]
    1632:	7e63      	ldrb	r3, [r4, #25]
    1634:	409e      	lsls	r6, r3
    1636:	0033      	movs	r3, r6
    1638:	001e      	movs	r6, r3
    163a:	6bea      	ldr	r2, [r5, #60]	@ 0x3c
    163c:	4016      	ands	r6, r2
    163e:	4213      	tst	r3, r2
    1640:	d1fa      	bne.n	0x1638
    1642:	21e0      	movs	r1, #224	@ 0xe0
    1644:	6960      	ldr	r0, [r4, #20]
    1646:	01c9      	lsls	r1, r1, #7
    1648:	f7ff f924 	bl	0x894
    164c:	491f      	ldr	r1, [pc, #124]	@ (0x16cc)
    164e:	6920      	ldr	r0, [r4, #16]
    1650:	f7ff f920 	bl	0x894
    1654:	23ff      	movs	r3, #255	@ 0xff
    1656:	221f      	movs	r2, #31
    1658:	602b      	str	r3, [r5, #0]
    165a:	9b01      	ldr	r3, [sp, #4]
    165c:	3310      	adds	r3, #16
    165e:	4013      	ands	r3, r2
    1660:	3a1e      	subs	r2, #30
    1662:	0011      	movs	r1, r2
    1664:	4099      	lsls	r1, r3
    1666:	4b14      	ldr	r3, [pc, #80]	@ (0x16b8)
    1668:	6019      	str	r1, [r3, #0]
    166a:	63fe      	str	r6, [r7, #60]	@ 0x3c
    166c:	643a      	str	r2, [r7, #64]	@ 0x40
    166e:	bdf7      	pop	{r0, r1, r2, r4, r5, r6, r7, pc}
    1670:	0011      	movs	r1, r2
    1672:	315c      	adds	r1, #92	@ 0x5c
    1674:	428b      	cmp	r3, r1
    1676:	d106      	bne.n	0x1686
    1678:	21c4      	movs	r1, #196	@ 0xc4
    167a:	480f      	ldr	r0, [pc, #60]	@ (0x16b8)
    167c:	0089      	lsls	r1, r1, #2
    167e:	5842      	ldr	r2, [r0, r1]
    1680:	4d13      	ldr	r5, [pc, #76]	@ (0x16d0)
    1682:	402a      	ands	r2, r5
    1684:	e7ae      	b.n	0x15e4
    1686:	0011      	movs	r1, r2
    1688:	316c      	adds	r1, #108	@ 0x6c
    168a:	428b      	cmp	r3, r1
    168c:	d105      	bne.n	0x169a
    168e:	21c4      	movs	r1, #196	@ 0xc4
    1690:	4809      	ldr	r0, [pc, #36]	@ (0x16b8)
    1692:	0089      	lsls	r1, r1, #2
    1694:	5842      	ldr	r2, [r0, r1]
    1696:	4d0f      	ldr	r5, [pc, #60]	@ (0x16d4)
    1698:	e7f3      	b.n	0x1682
    169a:	3274      	adds	r2, #116	@ 0x74
    169c:	4293      	cmp	r3, r2
    169e:	d1a2      	bne.n	0x15e6
    16a0:	21c4      	movs	r1, #196	@ 0xc4
    16a2:	4805      	ldr	r0, [pc, #20]	@ (0x16b8)
    16a4:	0089      	lsls	r1, r1, #2
    16a6:	5842      	ldr	r2, [r0, r1]
    16a8:	0212      	lsls	r2, r2, #8
    16aa:	0a12      	lsrs	r2, r2, #8
    16ac:	e79a      	b.n	0x15e4
    16ae:	46c0      	nop			@ (mov r8, r8)
    16b0:	00d4      	lsls	r4, r2, #3
    16b2:	1000      	asrs	r0, r0, #32
    16b4:	0070      	lsls	r0, r6, #1
    16b6:	1000      	asrs	r0, r0, #32
    16b8:	e100      	b.n	0x18bc
    16ba:	e000      	b.n	0x16be
    16bc:	8004      	strh	r4, [r0, #0]
    16be:	4004      	ands	r4, r0
    16c0:	4240      	negs	r0, r0
    16c2:	000f      	movs	r7, r1
    16c4:	001c      	movs	r4, r3
    16c6:	1000      	asrs	r0, r0, #32
    16c8:	ffff 0000 	vaddl.u<illegal width 64>	q8, d15, d0
    16cc:	2020      	movs	r0, #32
    16ce:	0000      	movs	r0, r0
    16d0:	00ff      	lsls	r7, r7, #3
    16d2:	ffff ffff 			@ <UNDEFINED> instruction: 0xffffffff
    16d6:	ff00 b5f8 	vqrshl.u8	<illegal reg q5.5>, q12, q8
    16da:	2001      	movs	r0, #1
    16dc:	f001 fae2 	bl	0x2ca4
    16e0:	2517      	movs	r5, #23
    16e2:	4c63      	ldr	r4, [pc, #396]	@ (0x1870)
    16e4:	7005      	strb	r5, [r0, #0]
    16e6:	67e0      	str	r0, [r4, #124]	@ 0x7c
    16e8:	2001      	movs	r0, #1
    16ea:	f001 fadb 	bl	0x2ca4
    16ee:	21c0      	movs	r1, #192	@ 0xc0
    16f0:	7005      	strb	r5, [r0, #0]
    16f2:	4d60      	ldr	r5, [pc, #384]	@ (0x1874)
    16f4:	01c9      	lsls	r1, r1, #7
    16f6:	6028      	str	r0, [r5, #0]
    16f8:	485f      	ldr	r0, [pc, #380]	@ (0x1878)
    16fa:	f7ff f8cb 	bl	0x894
    16fe:	26a0      	movs	r6, #160	@ 0xa0
    1700:	2380      	movs	r3, #128	@ 0x80
    1702:	2700      	movs	r7, #0
    1704:	009b      	lsls	r3, r3, #2
    1706:	05f6      	lsls	r6, r6, #23
    1708:	50f7      	str	r7, [r6, r3]
    170a:	f7ff fef9 	bl	0x1500
    170e:	21c0      	movs	r1, #192	@ 0xc0
    1710:	485a      	ldr	r0, [pc, #360]	@ (0x187c)
    1712:	01c9      	lsls	r1, r1, #7
    1714:	f7ff f8be 	bl	0x894
    1718:	2180      	movs	r1, #128	@ 0x80
    171a:	0049      	lsls	r1, r1, #1
    171c:	5077      	str	r7, [r6, r1]
    171e:	f7ff feef 	bl	0x1500
    1722:	2280      	movs	r2, #128	@ 0x80
    1724:	2380      	movs	r3, #128	@ 0x80
    1726:	0092      	lsls	r2, r2, #2
    1728:	50b3      	str	r3, [r6, r2]
    172a:	f7ff fee9 	bl	0x1500
    172e:	2180      	movs	r1, #128	@ 0x80
    1730:	2340      	movs	r3, #64	@ 0x40
    1732:	222e      	movs	r2, #46	@ 0x2e
    1734:	0020      	movs	r0, r4
    1736:	0049      	lsls	r1, r1, #1
    1738:	5073      	str	r3, [r6, r1]
    173a:	6be3      	ldr	r3, [r4, #60]	@ 0x3c
    173c:	300c      	adds	r0, #12
    173e:	751a      	strb	r2, [r3, #20]
    1740:	6be3      	ldr	r3, [r4, #60]	@ 0x3c
    1742:	3a2d      	subs	r2, #45	@ 0x2d
    1744:	755a      	strb	r2, [r3, #21]
    1746:	f7ff ff35 	bl	0x15b4
    174a:	217c      	movs	r1, #124	@ 0x7c
    174c:	4b4c      	ldr	r3, [pc, #304]	@ (0x1880)
    174e:	6be2      	ldr	r2, [r4, #60]	@ 0x3c
    1750:	8423      	strh	r3, [r4, #32]
    1752:	6b63      	ldr	r3, [r4, #52]	@ 0x34
    1754:	4249      	negs	r1, r1
    1756:	621f      	str	r7, [r3, #32]
    1758:	629f      	str	r7, [r3, #40]	@ 0x28
    175a:	7d13      	ldrb	r3, [r2, #20]
    175c:	404b      	eors	r3, r1
    175e:	7513      	strb	r3, [r2, #20]
    1760:	6be3      	ldr	r3, [r4, #60]	@ 0x3c
    1762:	7d1a      	ldrb	r2, [r3, #20]
    1764:	0750      	lsls	r0, r2, #29
    1766:	d500      	bpl.n	0x176a
    1768:	e071      	b.n	0x184e
    176a:	268c      	movs	r6, #140	@ 0x8c
    176c:	4f45      	ldr	r7, [pc, #276]	@ (0x1884)
    176e:	0076      	lsls	r6, r6, #1
    1770:	5dbb      	ldrb	r3, [r7, r6]
    1772:	2b00      	cmp	r3, #0
    1774:	d000      	beq.n	0x1778
    1776:	e06d      	b.n	0x1854
    1778:	4943      	ldr	r1, [pc, #268]	@ (0x1888)
    177a:	4844      	ldr	r0, [pc, #272]	@ (0x188c)
    177c:	f7ff f88a 	bl	0x894
    1780:	4943      	ldr	r1, [pc, #268]	@ (0x1890)
    1782:	4844      	ldr	r0, [pc, #272]	@ (0x1894)
    1784:	f7ff f886 	bl	0x894
    1788:	f3bf 8f4f 	dsb	sy
    178c:	f3bf 8f6f 	isb	sy
    1790:	2180      	movs	r1, #128	@ 0x80
    1792:	2280      	movs	r2, #128	@ 0x80
    1794:	4b40      	ldr	r3, [pc, #256]	@ (0x1898)
    1796:	0389      	lsls	r1, r1, #14
    1798:	5099      	str	r1, [r3, r2]
    179a:	2180      	movs	r1, #128	@ 0x80
    179c:	4b3f      	ldr	r3, [pc, #252]	@ (0x189c)
    179e:	0149      	lsls	r1, r1, #5
    17a0:	5898      	ldr	r0, [r3, r2]
    17a2:	4e3f      	ldr	r6, [pc, #252]	@ (0x18a0)
    17a4:	4301      	orrs	r1, r0
    17a6:	5099      	str	r1, [r3, r2]
    17a8:	2101      	movs	r1, #1
    17aa:	3218      	adds	r2, #24
    17ac:	5099      	str	r1, [r3, r2]
    17ae:	319a      	adds	r1, #154	@ 0x9a
    17b0:	60f1      	str	r1, [r6, #12]
    17b2:	6f98      	ldr	r0, [r3, #120]	@ 0x78
    17b4:	5899      	ldr	r1, [r3, r2]
    17b6:	4684      	mov	ip, r0
    17b8:	4b3a      	ldr	r3, [pc, #232]	@ (0x18a4)
    17ba:	6818      	ldr	r0, [r3, #0]
    17bc:	4663      	mov	r3, ip
    17be:	4358      	muls	r0, r3
    17c0:	f001 f9da 	bl	0x2b78
    17c4:	2196      	movs	r1, #150	@ 0x96
    17c6:	0900      	lsrs	r0, r0, #4
    17c8:	01c9      	lsls	r1, r1, #7
    17ca:	f001 f9d5 	bl	0x2b78
    17ce:	0a03      	lsrs	r3, r0, #8
    17d0:	6073      	str	r3, [r6, #4]
    17d2:	23ff      	movs	r3, #255	@ 0xff
    17d4:	4003      	ands	r3, r0
    17d6:	6033      	str	r3, [r6, #0]
    17d8:	231b      	movs	r3, #27
    17da:	60f3      	str	r3, [r6, #12]
    17dc:	3b14      	subs	r3, #20
    17de:	60b3      	str	r3, [r6, #8]
    17e0:	2300      	movs	r3, #0
    17e2:	2101      	movs	r1, #1
    17e4:	6133      	str	r3, [r6, #16]
    17e6:	6872      	ldr	r2, [r6, #4]
    17e8:	0038      	movs	r0, r7
    17ea:	430a      	orrs	r2, r1
    17ec:	6072      	str	r2, [r6, #4]
    17ee:	60bb      	str	r3, [r7, #8]
    17f0:	60fb      	str	r3, [r7, #12]
    17f2:	613b      	str	r3, [r7, #16]
    17f4:	617b      	str	r3, [r7, #20]
    17f6:	f7fe fd2f 	bl	0x258
    17fa:	2101      	movs	r1, #1
    17fc:	6973      	ldr	r3, [r6, #20]
    17fe:	4219      	tst	r1, r3
    1800:	d004      	beq.n	0x180c
    1802:	2201      	movs	r2, #1
    1804:	6833      	ldr	r3, [r6, #0]
    1806:	6973      	ldr	r3, [r6, #20]
    1808:	421a      	tst	r2, r3
    180a:	d1fb      	bne.n	0x1804
    180c:	20c5      	movs	r0, #197	@ 0xc5
    180e:	4b22      	ldr	r3, [pc, #136]	@ (0x1898)
    1810:	0080      	lsls	r0, r0, #2
    1812:	5819      	ldr	r1, [r3, r0]
    1814:	4a24      	ldr	r2, [pc, #144]	@ (0x18a8)
    1816:	4011      	ands	r1, r2
    1818:	22c0      	movs	r2, #192	@ 0xc0
    181a:	0212      	lsls	r2, r2, #8
    181c:	430a      	orrs	r2, r1
    181e:	501a      	str	r2, [r3, r0]
    1820:	2280      	movs	r2, #128	@ 0x80
    1822:	0392      	lsls	r2, r2, #14
    1824:	601a      	str	r2, [r3, #0]
    1826:	238c      	movs	r3, #140	@ 0x8c
    1828:	2201      	movs	r2, #1
    182a:	005b      	lsls	r3, r3, #1
    182c:	54fa      	strb	r2, [r7, r3]
    182e:	712a      	strb	r2, [r5, #4]
    1830:	2300      	movs	r3, #0
    1832:	2200      	movs	r2, #0
    1834:	0020      	movs	r0, r4
    1836:	60bb      	str	r3, [r7, #8]
    1838:	716b      	strb	r3, [r5, #5]
    183a:	60fb      	str	r3, [r7, #12]
    183c:	812b      	strh	r3, [r5, #8]
    183e:	613b      	str	r3, [r7, #16]
    1840:	60eb      	str	r3, [r5, #12]
    1842:	617b      	str	r3, [r7, #20]
    1844:	742a      	strb	r2, [r5, #16]
    1846:	616b      	str	r3, [r5, #20]
    1848:	61ab      	str	r3, [r5, #24]
    184a:	300c      	adds	r0, #12
    184c:	bdf8      	pop	{r3, r4, r5, r6, r7, pc}
    184e:	4051      	eors	r1, r2
    1850:	7519      	strb	r1, [r3, #20]
    1852:	e78a      	b.n	0x176a
    1854:	f7fe ff09 	bl	0x66a
    1858:	5dbe      	ldrb	r6, [r7, r6]
    185a:	490b      	ldr	r1, [pc, #44]	@ (0x1888)
    185c:	480b      	ldr	r0, [pc, #44]	@ (0x188c)
    185e:	f7ff f819 	bl	0x894
    1862:	2e00      	cmp	r6, #0
    1864:	d100      	bne.n	0x1868
    1866:	e78b      	b.n	0x1780
    1868:	f7fe feff 	bl	0x66a
    186c:	e788      	b.n	0x1780
    186e:	46c0      	nop			@ (mov r8, r8)
    1870:	0070      	lsls	r0, r6, #1
    1872:	1000      	asrs	r0, r0, #32
    1874:	00f0      	lsls	r0, r6, #3
    1876:	1000      	asrs	r0, r0, #32
    1878:	0207      	lsls	r7, r0, #8
    187a:	0005      	movs	r5, r0
    187c:	0206      	lsls	r6, r0, #8
    187e:	0070      	lsls	r0, r6, #1
    1880:	11fe      	asrs	r6, r7, #7
    1882:	0000      	movs	r0, r0
    1884:	0194      	lsls	r4, r2, #6
    1886:	1000      	asrs	r0, r0, #32
    1888:	6000      	str	r0, [r0, #0]
    188a:	0040      	lsls	r0, r0, #1
    188c:	8260      	strh	r0, [r4, #18]
    188e:	1034      	asrs	r4, r6, #32
    1890:	1000      	asrs	r0, r0, #32
    1892:	003c      	movs	r4, r7
    1894:	c261      	stmia	r2!, {r0, r5, r6}
    1896:	0f34      	lsrs	r4, r6, #28
    1898:	e100      	b.n	0x1a9c
    189a:	e000      	b.n	0x189e
    189c:	8000      	strh	r0, [r0, #0]
    189e:	4004      	ands	r4, r0
    18a0:	8000      	strh	r0, [r0, #0]
    18a2:	4000      	ands	r0, r0
    18a4:	001c      	movs	r4, r3
    18a6:	1000      	asrs	r0, r0, #32
    18a8:	00ff      	lsls	r7, r7, #3
    18aa:	ffff b510 	vsli.32	d27, d0, #31
    18ae:	0004      	movs	r4, r0
    18b0:	1d43      	adds	r3, r0, #5
    18b2:	7fda      	ldrb	r2, [r3, #31]
    18b4:	2a00      	cmp	r2, #0
    18b6:	d1fc      	bne.n	0x18b2
    18b8:	3201      	adds	r2, #1
    18ba:	77da      	strb	r2, [r3, #31]
    18bc:	230f      	movs	r3, #15
    18be:	69e1      	ldr	r1, [r4, #28]
    18c0:	69a0      	ldr	r0, [r4, #24]
    18c2:	794a      	ldrb	r2, [r1, #5]
    18c4:	401a      	ands	r2, r3
    18c6:	3207      	adds	r2, #7
    18c8:	f001 faea 	bl	0x2ea0
    18cc:	0020      	movs	r0, r4
    18ce:	f7fe fd89 	bl	0x3e4
    18d2:	bd10      	pop	{r4, pc}
    18d4:	b570      	push	{r4, r5, r6, lr}
    18d6:	7903      	ldrb	r3, [r0, #4]
    18d8:	0004      	movs	r4, r0
    18da:	2b00      	cmp	r3, #0
    18dc:	d03c      	beq.n	0x1958
    18de:	1d43      	adds	r3, r0, #5
    18e0:	7fdb      	ldrb	r3, [r3, #31]
    18e2:	2b00      	cmp	r3, #0
    18e4:	d138      	bne.n	0x1958
    18e6:	7943      	ldrb	r3, [r0, #5]
    18e8:	2b00      	cmp	r3, #0
    18ea:	d035      	beq.n	0x1958
    18ec:	4d1c      	ldr	r5, [pc, #112]	@ (0x1960)
    18ee:	68c2      	ldr	r2, [r0, #12]
    18f0:	682b      	ldr	r3, [r5, #0]
    18f2:	1a9b      	subs	r3, r3, r2
    18f4:	4a1b      	ldr	r2, [pc, #108]	@ (0x1964)
    18f6:	4293      	cmp	r3, r2
    18f8:	d901      	bls.n	0x18fe
    18fa:	f7fe fdc1 	bl	0x480
    18fe:	7963      	ldrb	r3, [r4, #5]
    1900:	2b01      	cmp	r3, #1
    1902:	d112      	bne.n	0x192a
    1904:	1da3      	adds	r3, r4, #6
    1906:	7fda      	ldrb	r2, [r3, #31]
    1908:	2a01      	cmp	r2, #1
    190a:	d125      	bne.n	0x1958
    190c:	7fda      	ldrb	r2, [r3, #31]
    190e:	2a01      	cmp	r2, #1
    1910:	d122      	bne.n	0x1958
    1912:	3201      	adds	r2, #1
    1914:	7162      	strb	r2, [r4, #5]
    1916:	0020      	movs	r0, r4
    1918:	77da      	strb	r2, [r3, #31]
    191a:	f7ff ffc7 	bl	0x18ac
    191e:	2300      	movs	r3, #0
    1920:	72a3      	strb	r3, [r4, #10]
    1922:	682b      	ldr	r3, [r5, #0]
    1924:	6123      	str	r3, [r4, #16]
    1926:	682b      	ldr	r3, [r5, #0]
    1928:	60e3      	str	r3, [r4, #12]
    192a:	7963      	ldrb	r3, [r4, #5]
    192c:	2b02      	cmp	r3, #2
    192e:	d113      	bne.n	0x1958
    1930:	6922      	ldr	r2, [r4, #16]
    1932:	682b      	ldr	r3, [r5, #0]
    1934:	1a9b      	subs	r3, r3, r2
    1936:	4a0c      	ldr	r2, [pc, #48]	@ (0x1968)
    1938:	4293      	cmp	r3, r2
    193a:	d90d      	bls.n	0x1958
    193c:	230a      	movs	r3, #10
    193e:	56e3      	ldrsb	r3, [r4, r3]
    1940:	0020      	movs	r0, r4
    1942:	2b02      	cmp	r3, #2
    1944:	dc09      	bgt.n	0x195a
    1946:	f7ff ffb1 	bl	0x18ac
    194a:	7aa3      	ldrb	r3, [r4, #10]
    194c:	3301      	adds	r3, #1
    194e:	72a3      	strb	r3, [r4, #10]
    1950:	682b      	ldr	r3, [r5, #0]
    1952:	6123      	str	r3, [r4, #16]
    1954:	682b      	ldr	r3, [r5, #0]
    1956:	60e3      	str	r3, [r4, #12]
    1958:	bd70      	pop	{r4, r5, r6, pc}
    195a:	f7fe fd91 	bl	0x480
    195e:	e7fb      	b.n	0x1958
    1960:	0070      	lsls	r0, r6, #1
    1962:	1000      	asrs	r0, r0, #32
    1964:	176f      	asrs	r7, r5, #29
    1966:	0000      	movs	r0, r0
    1968:	0bb7      	lsrs	r7, r6, #14
    196a:	0000      	movs	r0, r0
    196c:	b5f0      	push	{r4, r5, r6, r7, lr}
    196e:	0004      	movs	r4, r0
    1970:	b085      	sub	sp, #20
    1972:	f7ff ffaf 	bl	0x18d4
    1976:	6aa3      	ldr	r3, [r4, #40]	@ 0x28
    1978:	685a      	ldr	r2, [r3, #4]
    197a:	2a00      	cmp	r2, #0
    197c:	d036      	beq.n	0x19ec
    197e:	6bda      	ldr	r2, [r3, #60]	@ 0x3c
    1980:	2a00      	cmp	r2, #0
    1982:	d133      	bne.n	0x19ec
    1984:	6ada      	ldr	r2, [r3, #44]	@ 0x2c
    1986:	2a00      	cmp	r2, #0
    1988:	d130      	bne.n	0x19ec
    198a:	6b22      	ldr	r2, [r4, #48]	@ 0x30
    198c:	7d12      	ldrb	r2, [r2, #20]
    198e:	0752      	lsls	r2, r2, #29
    1990:	d52c      	bpl.n	0x19ec
    1992:	20c0      	movs	r0, #192	@ 0xc0
    1994:	681d      	ldr	r5, [r3, #0]
    1996:	685b      	ldr	r3, [r3, #4]
    1998:	792a      	ldrb	r2, [r5, #4]
    199a:	b2db      	uxtb	r3, r3
    199c:	9302      	str	r3, [sp, #8]
    199e:	78eb      	ldrb	r3, [r5, #3]
    19a0:	0212      	lsls	r2, r2, #8
    19a2:	431a      	orrs	r2, r3
    19a4:	79ab      	ldrb	r3, [r5, #6]
    19a6:	79ee      	ldrb	r6, [r5, #7]
    19a8:	0219      	lsls	r1, r3, #8
    19aa:	0080      	lsls	r0, r0, #2
    19ac:	4001      	ands	r1, r0
    19ae:	430e      	orrs	r6, r1
    19b0:	21f0      	movs	r1, #240	@ 0xf0
    19b2:	27a0      	movs	r7, #160	@ 0xa0
    19b4:	0089      	lsls	r1, r1, #2
    19b6:	4031      	ands	r1, r6
    19b8:	00bf      	lsls	r7, r7, #2
    19ba:	42b9      	cmp	r1, r7
    19bc:	d008      	beq.n	0x19d0
    19be:	dc69      	bgt.n	0x1a94
    19c0:	0008      	movs	r0, r1
    19c2:	2780      	movs	r7, #128	@ 0x80
    19c4:	3881      	subs	r0, #129	@ 0x81
    19c6:	38ff      	subs	r0, #255	@ 0xff
    19c8:	43b8      	bics	r0, r7
    19ca:	b280      	uxth	r0, r0
    19cc:	2800      	cmp	r0, #0
    19ce:	d163      	bne.n	0x1a98
    19d0:	b28e      	uxth	r6, r1
    19d2:	2a00      	cmp	r2, #0
    19d4:	d162      	bne.n	0x1a9c
    19d6:	6823      	ldr	r3, [r4, #0]
    19d8:	002a      	movs	r2, r5
    19da:	699f      	ldr	r7, [r3, #24]
    19dc:	0031      	movs	r1, r6
    19de:	0020      	movs	r0, r4
    19e0:	9b02      	ldr	r3, [sp, #8]
    19e2:	47b8      	blx	r7
    19e4:	6823      	ldr	r3, [r4, #0]
    19e6:	0020      	movs	r0, r4
    19e8:	6a1b      	ldr	r3, [r3, #32]
    19ea:	4798      	blx	r3
    19ec:	6ae0      	ldr	r0, [r4, #44]	@ 0x2c
    19ee:	2800      	cmp	r0, #0
    19f0:	d038      	beq.n	0x1a64
    19f2:	49c5      	ldr	r1, [pc, #788]	@ (0x1d08)
    19f4:	f7fe ff4e 	bl	0x894
    19f8:	6ae2      	ldr	r2, [r4, #44]	@ 0x2c
    19fa:	4dc4      	ldr	r5, [pc, #784]	@ (0x1d0c)
    19fc:	0653      	lsls	r3, r2, #25
    19fe:	0f9b      	lsrs	r3, r3, #30
    1a00:	009b      	lsls	r3, r3, #2
    1a02:	5958      	ldr	r0, [r3, r5]
    1a04:	231f      	movs	r3, #31
    1a06:	401a      	ands	r2, r3
    1a08:	3b1b      	subs	r3, #27
    1a0a:	4093      	lsls	r3, r2
    1a0c:	581a      	ldr	r2, [r3, r0]
    1a0e:	6c60      	ldr	r0, [r4, #68]	@ 0x44
    1a10:	1e53      	subs	r3, r2, #1
    1a12:	419a      	sbcs	r2, r3
    1a14:	4bbe      	ldr	r3, [pc, #760]	@ (0x1d10)
    1a16:	6c21      	ldr	r1, [r4, #64]	@ 0x40
    1a18:	681b      	ldr	r3, [r3, #0]
    1a1a:	4282      	cmp	r2, r0
    1a1c:	d100      	bne.n	0x1a20
    1a1e:	e17e      	b.n	0x1d1e
    1a20:	63e3      	str	r3, [r4, #60]	@ 0x3c
    1a22:	6462      	str	r2, [r4, #68]	@ 0x44
    1a24:	6c23      	ldr	r3, [r4, #64]	@ 0x40
    1a26:	2b00      	cmp	r3, #0
    1a28:	d106      	bne.n	0x1a38
    1a2a:	2900      	cmp	r1, #0
    1a2c:	d004      	beq.n	0x1a38
    1a2e:	6b22      	ldr	r2, [r4, #48]	@ 0x30
    1a30:	3b7f      	subs	r3, #127	@ 0x7f
    1a32:	7d11      	ldrb	r1, [r2, #20]
    1a34:	404b      	eors	r3, r1
    1a36:	7513      	strb	r3, [r2, #20]
    1a38:	21c0      	movs	r1, #192	@ 0xc0
    1a3a:	6ae0      	ldr	r0, [r4, #44]	@ 0x2c
    1a3c:	01c9      	lsls	r1, r1, #7
    1a3e:	f7fe ff29 	bl	0x894
    1a42:	2001      	movs	r0, #1
    1a44:	221f      	movs	r2, #31
    1a46:	0001      	movs	r1, r0
    1a48:	6ae3      	ldr	r3, [r4, #44]	@ 0x2c
    1a4a:	401a      	ands	r2, r3
    1a4c:	4091      	lsls	r1, r2
    1a4e:	6b22      	ldr	r2, [r4, #48]	@ 0x30
    1a50:	065b      	lsls	r3, r3, #25
    1a52:	7d12      	ldrb	r2, [r2, #20]
    1a54:	0f9b      	lsrs	r3, r3, #30
    1a56:	4002      	ands	r2, r0
    1a58:	009b      	lsls	r3, r3, #2
    1a5a:	58eb      	ldr	r3, [r5, r3]
    1a5c:	3a01      	subs	r2, #1
    1a5e:	400a      	ands	r2, r1
    1a60:	0089      	lsls	r1, r1, #2
    1a62:	50ca      	str	r2, [r1, r3]
    1a64:	6aa3      	ldr	r3, [r4, #40]	@ 0x28
    1a66:	6bda      	ldr	r2, [r3, #60]	@ 0x3c
    1a68:	2a00      	cmp	r2, #0
    1a6a:	d111      	bne.n	0x1a90
    1a6c:	6ade      	ldr	r6, [r3, #44]	@ 0x2c
    1a6e:	2e00      	cmp	r6, #0
    1a70:	d10e      	bne.n	0x1a90
    1a72:	0023      	movs	r3, r4
    1a74:	3348      	adds	r3, #72	@ 0x48
    1a76:	781b      	ldrb	r3, [r3, #0]
    1a78:	2b00      	cmp	r3, #0
    1a7a:	d009      	beq.n	0x1a90
    1a7c:	0025      	movs	r5, r4
    1a7e:	3549      	adds	r5, #73	@ 0x49
    1a80:	782a      	ldrb	r2, [r5, #0]
    1a82:	2a00      	cmp	r2, #0
    1a84:	d100      	bne.n	0x1a88
    1a86:	e156      	b.n	0x1d36
    1a88:	0020      	movs	r0, r4
    1a8a:	f7fe fcf9 	bl	0x480
    1a8e:	702e      	strb	r6, [r5, #0]
    1a90:	b005      	add	sp, #20
    1a92:	bdf0      	pop	{r4, r5, r6, r7, pc}
    1a94:	4281      	cmp	r1, r0
    1a96:	d09b      	beq.n	0x19d0
    1a98:	b2b6      	uxth	r6, r6
    1a9a:	e79a      	b.n	0x19d2
    1a9c:	7969      	ldrb	r1, [r5, #5]
    1a9e:	ba52      	rev16	r2, r2
    1aa0:	b292      	uxth	r2, r2
    1aa2:	297f      	cmp	r1, #127	@ 0x7f
    1aa4:	d908      	bls.n	0x1ab8
    1aa6:	9902      	ldr	r1, [sp, #8]
    1aa8:	6823      	ldr	r3, [r4, #0]
    1aaa:	9100      	str	r1, [sp, #0]
    1aac:	695f      	ldr	r7, [r3, #20]
    1aae:	0031      	movs	r1, r6
    1ab0:	002b      	movs	r3, r5
    1ab2:	0020      	movs	r0, r4
    1ab4:	47b8      	blx	r7
    1ab6:	e795      	b.n	0x19e4
    1ab8:	8aa1      	ldrh	r1, [r4, #20]
    1aba:	4291      	cmp	r1, r2
    1abc:	d000      	beq.n	0x1ac0
    1abe:	e791      	b.n	0x19e4
    1ac0:	223c      	movs	r2, #60	@ 0x3c
    1ac2:	0018      	movs	r0, r3
    1ac4:	4390      	bics	r0, r2
    1ac6:	78aa      	ldrb	r2, [r5, #2]
    1ac8:	7869      	ldrb	r1, [r5, #1]
    1aca:	0212      	lsls	r2, r2, #8
    1acc:	430a      	orrs	r2, r1
    1ace:	ba57      	rev16	r7, r2
    1ad0:	b242      	sxtb	r2, r0
    1ad2:	b2b9      	uxth	r1, r7
    1ad4:	2a00      	cmp	r2, #0
    1ad6:	db00      	blt.n	0x1ada
    1ad8:	e08f      	b.n	0x1bfa
    1ada:	2880      	cmp	r0, #128	@ 0x80
    1adc:	d016      	beq.n	0x1b0c
    1ade:	2881      	cmp	r0, #129	@ 0x81
    1ae0:	d03a      	beq.n	0x1b58
    1ae2:	07da      	lsls	r2, r3, #31
    1ae4:	d465      	bmi.n	0x1bb2
    1ae6:	7962      	ldrb	r2, [r4, #5]
    1ae8:	2a01      	cmp	r2, #1
    1aea:	d800      	bhi.n	0x1aee
    1aec:	e77a      	b.n	0x19e4
    1aee:	88e2      	ldrh	r2, [r4, #6]
    1af0:	428a      	cmp	r2, r1
    1af2:	d000      	beq.n	0x1af6
    1af4:	e776      	b.n	0x19e4
    1af6:	2108      	movs	r1, #8
    1af8:	200f      	movs	r0, #15
    1afa:	5661      	ldrsb	r1, [r4, r1]
    1afc:	089b      	lsrs	r3, r3, #2
    1afe:	4003      	ands	r3, r0
    1b00:	428b      	cmp	r3, r1
    1b02:	d03d      	beq.n	0x1b80
    1b04:	0020      	movs	r0, r4
    1b06:	f7fe fcbb 	bl	0x480
    1b0a:	e76b      	b.n	0x19e4
    1b0c:	7965      	ldrb	r5, [r4, #5]
    1b0e:	2d00      	cmp	r5, #0
    1b10:	d012      	beq.n	0x1b38
    1b12:	3d01      	subs	r5, #1
    1b14:	2d01      	cmp	r5, #1
    1b16:	d900      	bls.n	0x1b1a
    1b18:	e764      	b.n	0x19e4
    1b1a:	88e3      	ldrh	r3, [r4, #6]
    1b1c:	428b      	cmp	r3, r1
    1b1e:	d100      	bne.n	0x1b22
    1b20:	e760      	b.n	0x19e4
    1b22:	4a7b      	ldr	r2, [pc, #492]	@ (0x1d10)
    1b24:	0020      	movs	r0, r4
    1b26:	8893      	ldrh	r3, [r2, #4]
    1b28:	3301      	adds	r3, #1
    1b2a:	8093      	strh	r3, [r2, #4]
    1b2c:	000a      	movs	r2, r1
    1b2e:	2300      	movs	r3, #0
    1b30:	2181      	movs	r1, #129	@ 0x81
    1b32:	f7fe fc89 	bl	0x448
    1b36:	e755      	b.n	0x19e4
    1b38:	6823      	ldr	r3, [r4, #0]
    1b3a:	0020      	movs	r0, r4
    1b3c:	6a9b      	ldr	r3, [r3, #40]	@ 0x28
    1b3e:	4798      	blx	r3
    1b40:	4b73      	ldr	r3, [pc, #460]	@ (0x1d10)
    1b42:	80e7      	strh	r7, [r4, #6]
    1b44:	681b      	ldr	r3, [r3, #0]
    1b46:	8125      	strh	r5, [r4, #8]
    1b48:	60e3      	str	r3, [r4, #12]
    1b4a:	7963      	ldrb	r3, [r4, #5]
    1b4c:	2b01      	cmp	r3, #1
    1b4e:	d100      	bne.n	0x1b52
    1b50:	e748      	b.n	0x19e4
    1b52:	2301      	movs	r3, #1
    1b54:	7163      	strb	r3, [r4, #5]
    1b56:	e745      	b.n	0x19e4
    1b58:	7963      	ldrb	r3, [r4, #5]
    1b5a:	6820      	ldr	r0, [r4, #0]
    1b5c:	3b01      	subs	r3, #1
    1b5e:	2b01      	cmp	r3, #1
    1b60:	d900      	bls.n	0x1b64
    1b62:	e73f      	b.n	0x19e4
    1b64:	88e3      	ldrh	r3, [r4, #6]
    1b66:	428b      	cmp	r3, r1
    1b68:	d000      	beq.n	0x1b6c
    1b6a:	e73b      	b.n	0x19e4
    1b6c:	2300      	movs	r3, #0
    1b6e:	4a68      	ldr	r2, [pc, #416]	@ (0x1d10)
    1b70:	7163      	strb	r3, [r4, #5]
    1b72:	8893      	ldrh	r3, [r2, #4]
    1b74:	3301      	adds	r3, #1
    1b76:	8093      	strh	r3, [r2, #4]
    1b78:	6a83      	ldr	r3, [r0, #40]	@ 0x28
    1b7a:	0020      	movs	r0, r4
    1b7c:	4798      	blx	r3
    1b7e:	e731      	b.n	0x19e4
    1b80:	2600      	movs	r6, #0
    1b82:	4b63      	ldr	r3, [pc, #396]	@ (0x1d10)
    1b84:	3101      	adds	r1, #1
    1b86:	681b      	ldr	r3, [r3, #0]
    1b88:	4001      	ands	r1, r0
    1b8a:	1da7      	adds	r7, r4, #6
    1b8c:	7221      	strb	r1, [r4, #8]
    1b8e:	60e3      	str	r3, [r4, #12]
    1b90:	1de5      	adds	r5, r4, #7
    1b92:	77fe      	strb	r6, [r7, #31]
    1b94:	7feb      	ldrb	r3, [r5, #31]
    1b96:	42b3      	cmp	r3, r6
    1b98:	d0db      	beq.n	0x1b52
    1b9a:	6a21      	ldr	r1, [r4, #32]
    1b9c:	794a      	ldrb	r2, [r1, #5]
    1b9e:	4002      	ands	r2, r0
    1ba0:	3207      	adds	r2, #7
    1ba2:	69e0      	ldr	r0, [r4, #28]
    1ba4:	f001 f97c 	bl	0x2ea0
    1ba8:	7feb      	ldrb	r3, [r5, #31]
    1baa:	b2db      	uxtb	r3, r3
    1bac:	77fb      	strb	r3, [r7, #31]
    1bae:	77ee      	strb	r6, [r5, #31]
    1bb0:	e7cb      	b.n	0x1b4a
    1bb2:	2bff      	cmp	r3, #255	@ 0xff
    1bb4:	d100      	bne.n	0x1bb8
    1bb6:	e715      	b.n	0x19e4
    1bb8:	7962      	ldrb	r2, [r4, #5]
    1bba:	2a00      	cmp	r2, #0
    1bbc:	d100      	bne.n	0x1bc0
    1bbe:	e711      	b.n	0x19e4
    1bc0:	2008      	movs	r0, #8
    1bc2:	5620      	ldrsb	r0, [r4, r0]
    1bc4:	069b      	lsls	r3, r3, #26
    1bc6:	0f1b      	lsrs	r3, r3, #28
    1bc8:	4283      	cmp	r3, r0
    1bca:	d000      	beq.n	0x1bce
    1bcc:	e70a      	b.n	0x19e4
    1bce:	88e3      	ldrh	r3, [r4, #6]
    1bd0:	428b      	cmp	r3, r1
    1bd2:	d000      	beq.n	0x1bd6
    1bd4:	e706      	b.n	0x19e4
    1bd6:	230a      	movs	r3, #10
    1bd8:	56e3      	ldrsb	r3, [r4, r3]
    1bda:	0020      	movs	r0, r4
    1bdc:	2b02      	cmp	r3, #2
    1bde:	dc92      	bgt.n	0x1b06
    1be0:	2a02      	cmp	r2, #2
    1be2:	d190      	bne.n	0x1b06
    1be4:	f7ff fe62 	bl	0x18ac
    1be8:	7aa3      	ldrb	r3, [r4, #10]
    1bea:	3301      	adds	r3, #1
    1bec:	72a3      	strb	r3, [r4, #10]
    1bee:	4b48      	ldr	r3, [pc, #288]	@ (0x1d10)
    1bf0:	681a      	ldr	r2, [r3, #0]
    1bf2:	681b      	ldr	r3, [r3, #0]
    1bf4:	6122      	str	r2, [r4, #16]
    1bf6:	60e3      	str	r3, [r4, #12]
    1bf8:	e6f4      	b.n	0x19e4
    1bfa:	2b03      	cmp	r3, #3
    1bfc:	d821      	bhi.n	0x1c42
    1bfe:	1d67      	adds	r7, r4, #5
    1c00:	7ffb      	ldrb	r3, [r7, #31]
    1c02:	2b00      	cmp	r3, #0
    1c04:	d1fc      	bne.n	0x1c00
    1c06:	220c      	movs	r2, #12
    1c08:	3301      	adds	r3, #1
    1c0a:	77fb      	strb	r3, [r7, #31]
    1c0c:	7828      	ldrb	r0, [r5, #0]
    1c0e:	69a3      	ldr	r3, [r4, #24]
    1c10:	4010      	ands	r0, r2
    1c12:	3a5c      	subs	r2, #92	@ 0x5c
    1c14:	4302      	orrs	r2, r0
    1c16:	701a      	strb	r2, [r3, #0]
    1c18:	0a0a      	lsrs	r2, r1, #8
    1c1a:	7119      	strb	r1, [r3, #4]
    1c1c:	70da      	strb	r2, [r3, #3]
    1c1e:	6822      	ldr	r2, [r4, #0]
    1c20:	9300      	str	r3, [sp, #0]
    1c22:	69d3      	ldr	r3, [r2, #28]
    1c24:	0031      	movs	r1, r6
    1c26:	469c      	mov	ip, r3
    1c28:	002a      	movs	r2, r5
    1c2a:	0020      	movs	r0, r4
    1c2c:	4665      	mov	r5, ip
    1c2e:	9b02      	ldr	r3, [sp, #8]
    1c30:	47a8      	blx	r5
    1c32:	2800      	cmp	r0, #0
    1c34:	d003      	beq.n	0x1c3e
    1c36:	0020      	movs	r0, r4
    1c38:	f7fe fbd4 	bl	0x3e4
    1c3c:	e6d2      	b.n	0x19e4
    1c3e:	77f8      	strb	r0, [r7, #31]
    1c40:	e6d0      	b.n	0x19e4
    1c42:	88e2      	ldrh	r2, [r4, #6]
    1c44:	428a      	cmp	r2, r1
    1c46:	d000      	beq.n	0x1c4a
    1c48:	e6cc      	b.n	0x19e4
    1c4a:	7961      	ldrb	r1, [r4, #5]
    1c4c:	2900      	cmp	r1, #0
    1c4e:	d100      	bne.n	0x1c52
    1c50:	e6c8      	b.n	0x19e4
    1c52:	089b      	lsrs	r3, r3, #2
    1c54:	2109      	movs	r1, #9
    1c56:	0018      	movs	r0, r3
    1c58:	270f      	movs	r7, #15
    1c5a:	5661      	ldrsb	r1, [r4, r1]
    1c5c:	4038      	ands	r0, r7
    1c5e:	0003      	movs	r3, r0
    1c60:	4288      	cmp	r0, r1
    1c62:	d143      	bne.n	0x1cec
    1c64:	21c2      	movs	r1, #194	@ 0xc2
    1c66:	0020      	movs	r0, r4
    1c68:	f7fe fbee 	bl	0x448
    1c6c:	7a63      	ldrb	r3, [r4, #9]
    1c6e:	3301      	adds	r3, #1
    1c70:	403b      	ands	r3, r7
    1c72:	7263      	strb	r3, [r4, #9]
    1c74:	4b26      	ldr	r3, [pc, #152]	@ (0x1d10)
    1c76:	681b      	ldr	r3, [r3, #0]
    1c78:	60e3      	str	r3, [r4, #12]
    1c7a:	1da3      	adds	r3, r4, #6
    1c7c:	7fdb      	ldrb	r3, [r3, #31]
    1c7e:	2b00      	cmp	r3, #0
    1c80:	d12d      	bne.n	0x1cde
    1c82:	0023      	movs	r3, r4
    1c84:	69e7      	ldr	r7, [r4, #28]
    1c86:	3325      	adds	r3, #37	@ 0x25
    1c88:	9303      	str	r3, [sp, #12]
    1c8a:	2301      	movs	r3, #1
    1c8c:	9a03      	ldr	r2, [sp, #12]
    1c8e:	0031      	movs	r1, r6
    1c90:	7013      	strb	r3, [r2, #0]
    1c92:	6823      	ldr	r3, [r4, #0]
    1c94:	9700      	str	r7, [sp, #0]
    1c96:	69db      	ldr	r3, [r3, #28]
    1c98:	002a      	movs	r2, r5
    1c9a:	469c      	mov	ip, r3
    1c9c:	0020      	movs	r0, r4
    1c9e:	4666      	mov	r6, ip
    1ca0:	9b02      	ldr	r3, [sp, #8]
    1ca2:	47b0      	blx	r6
    1ca4:	2800      	cmp	r0, #0
    1ca6:	d01e      	beq.n	0x1ce6
    1ca8:	230c      	movs	r3, #12
    1caa:	782a      	ldrb	r2, [r5, #0]
    1cac:	401a      	ands	r2, r3
    1cae:	3b5c      	subs	r3, #92	@ 0x5c
    1cb0:	4313      	orrs	r3, r2
    1cb2:	703b      	strb	r3, [r7, #0]
    1cb4:	88e3      	ldrh	r3, [r4, #6]
    1cb6:	0a1a      	lsrs	r2, r3, #8
    1cb8:	713b      	strb	r3, [r7, #4]
    1cba:	2308      	movs	r3, #8
    1cbc:	70fa      	strb	r2, [r7, #3]
    1cbe:	69e2      	ldr	r2, [r4, #28]
    1cc0:	56e3      	ldrsb	r3, [r4, r3]
    1cc2:	4297      	cmp	r7, r2
    1cc4:	d002      	beq.n	0x1ccc
    1cc6:	220f      	movs	r2, #15
    1cc8:	3301      	adds	r3, #1
    1cca:	4013      	ands	r3, r2
    1ccc:	213c      	movs	r1, #60	@ 0x3c
    1cce:	79ba      	ldrb	r2, [r7, #6]
    1cd0:	009b      	lsls	r3, r3, #2
    1cd2:	438a      	bics	r2, r1
    1cd4:	4313      	orrs	r3, r2
    1cd6:	2240      	movs	r2, #64	@ 0x40
    1cd8:	4313      	orrs	r3, r2
    1cda:	71bb      	strb	r3, [r7, #6]
    1cdc:	e682      	b.n	0x19e4
    1cde:	0023      	movs	r3, r4
    1ce0:	6a27      	ldr	r7, [r4, #32]
    1ce2:	3326      	adds	r3, #38	@ 0x26
    1ce4:	e7d0      	b.n	0x1c88
    1ce6:	9b03      	ldr	r3, [sp, #12]
    1ce8:	7018      	strb	r0, [r3, #0]
    1cea:	e67b      	b.n	0x19e4
    1cec:	3901      	subs	r1, #1
    1cee:	4d08      	ldr	r5, [pc, #32]	@ (0x1d10)
    1cf0:	4039      	ands	r1, r7
    1cf2:	4288      	cmp	r0, r1
    1cf4:	d10e      	bne.n	0x1d14
    1cf6:	21c2      	movs	r1, #194	@ 0xc2
    1cf8:	0020      	movs	r0, r4
    1cfa:	f7fe fba5 	bl	0x448
    1cfe:	88eb      	ldrh	r3, [r5, #6]
    1d00:	3301      	adds	r3, #1
    1d02:	80eb      	strh	r3, [r5, #6]
    1d04:	682b      	ldr	r3, [r5, #0]
    1d06:	e776      	b.n	0x1bf6
    1d08:	1010      	asrs	r0, r2, #32
    1d0a:	0000      	movs	r0, r0
    1d0c:	2ff8      	cmp	r7, #248	@ 0xf8
    1d0e:	0000      	movs	r0, r0
    1d10:	0070      	lsls	r0, r6, #1
    1d12:	1000      	asrs	r0, r0, #32
    1d14:	21c3      	movs	r1, #195	@ 0xc3
    1d16:	0020      	movs	r0, r4
    1d18:	f7fe fb96 	bl	0x448
    1d1c:	e7f2      	b.n	0x1d04
    1d1e:	6be0      	ldr	r0, [r4, #60]	@ 0x3c
    1d20:	2800      	cmp	r0, #0
    1d22:	d100      	bne.n	0x1d26
    1d24:	e67e      	b.n	0x1a24
    1d26:	1a1b      	subs	r3, r3, r0
    1d28:	2b32      	cmp	r3, #50	@ 0x32
    1d2a:	d500      	bpl.n	0x1d2e
    1d2c:	e67a      	b.n	0x1a24
    1d2e:	2300      	movs	r3, #0
    1d30:	6422      	str	r2, [r4, #64]	@ 0x40
    1d32:	63e3      	str	r3, [r4, #60]	@ 0x3c
    1d34:	e676      	b.n	0x1a24
    1d36:	6ce1      	ldr	r1, [r4, #76]	@ 0x4c
    1d38:	2900      	cmp	r1, #0
    1d3a:	d100      	bne.n	0x1d3e
    1d3c:	e6a8      	b.n	0x1a90
    1d3e:	4b05      	ldr	r3, [pc, #20]	@ (0x1d54)
    1d40:	681b      	ldr	r3, [r3, #0]
    1d42:	1a5b      	subs	r3, r3, r1
    1d44:	d500      	bpl.n	0x1d48
    1d46:	e6a3      	b.n	0x1a90
    1d48:	6823      	ldr	r3, [r4, #0]
    1d4a:	0020      	movs	r0, r4
    1d4c:	6b1b      	ldr	r3, [r3, #48]	@ 0x30
    1d4e:	64e2      	str	r2, [r4, #76]	@ 0x4c
    1d50:	4798      	blx	r3
    1d52:	e69d      	b.n	0x1a90
    1d54:	0070      	lsls	r0, r6, #1
    1d56:	1000      	asrs	r0, r0, #32
    1d58:	b5f0      	push	{r4, r5, r6, r7, lr}
    1d5a:	780c      	ldrb	r4, [r1, #0]
    1d5c:	b085      	sub	sp, #20
    1d5e:	2c29      	cmp	r4, #41	@ 0x29
    1d60:	d103      	bne.n	0x1d6a
    1d62:	7814      	ldrb	r4, [r2, #0]
    1d64:	2c0a      	cmp	r4, #10
    1d66:	d100      	bne.n	0x1d6a
    1d68:	e0a5      	b.n	0x1eb6
    1d6a:	4c6a      	ldr	r4, [pc, #424]	@ (0x1f14)
    1d6c:	2520      	movs	r5, #32
    1d6e:	26d0      	movs	r6, #208	@ 0xd0
    1d70:	5566      	strb	r6, [r4, r5]
    1d72:	781b      	ldrb	r3, [r3, #0]
    1d74:	425e      	negs	r6, r3
    1d76:	4173      	adcs	r3, r6
    1d78:	261f      	movs	r6, #31
    1d7a:	425b      	negs	r3, r3
    1d7c:	43b3      	bics	r3, r6
    1d7e:	33f0      	adds	r3, #240	@ 0xf0
    1d80:	3649      	adds	r6, #73	@ 0x49
    1d82:	5563      	strb	r3, [r4, r5]
    1d84:	7726      	strb	r6, [r4, #28]
    1d86:	7817      	ldrb	r7, [r2, #0]
    1d88:	77e6      	strb	r6, [r4, #31]
    1d8a:	7767      	strb	r7, [r4, #29]
    1d8c:	77a7      	strb	r7, [r4, #30]
    1d8e:	7806      	ldrb	r6, [r0, #0]
    1d90:	200f      	movs	r0, #15
    1d92:	4030      	ands	r0, r6
    1d94:	4303      	orrs	r3, r0
    1d96:	5563      	strb	r3, [r4, r5]
    1d98:	2321      	movs	r3, #33	@ 0x21
    1d9a:	7809      	ldrb	r1, [r1, #0]
    1d9c:	54e1      	strb	r1, [r4, r3]
    1d9e:	7815      	ldrb	r5, [r2, #0]
    1da0:	1960      	adds	r0, r4, r5
    1da2:	3020      	adds	r0, #32
    1da4:	2d00      	cmp	r5, #0
    1da6:	d100      	bne.n	0x1daa
    1da8:	e0aa      	b.n	0x1f00
    1daa:	1e6b      	subs	r3, r5, #1
    1dac:	2b15      	cmp	r3, #21
    1dae:	d800      	bhi.n	0x1db2
    1db0:	e0ac      	b.n	0x1f0c
    1db2:	08ab      	lsrs	r3, r5, #2
    1db4:	469c      	mov	ip, r3
    1db6:	9203      	str	r2, [sp, #12]
    1db8:	0022      	movs	r2, r4
    1dba:	0027      	movs	r7, r4
    1dbc:	2100      	movs	r1, #0
    1dbe:	4664      	mov	r4, ip
    1dc0:	2300      	movs	r3, #0
    1dc2:	4694      	mov	ip, r2
    1dc4:	4e54      	ldr	r6, [pc, #336]	@ (0x1f18)
    1dc6:	3720      	adds	r7, #32
    1dc8:	9501      	str	r5, [sp, #4]
    1dca:	9002      	str	r0, [sp, #8]
    1dcc:	cf04      	ldmia	r7!, {r2}
    1dce:	0018      	movs	r0, r3
    1dd0:	0015      	movs	r5, r2
    1dd2:	4030      	ands	r0, r6
    1dd4:	405a      	eors	r2, r3
    1dd6:	4035      	ands	r5, r6
    1dd8:	4b50      	ldr	r3, [pc, #320]	@ (0x1f1c)
    1dda:	1940      	adds	r0, r0, r5
    1ddc:	401a      	ands	r2, r3
    1dde:	0003      	movs	r3, r0
    1de0:	3101      	adds	r1, #1
    1de2:	b2c9      	uxtb	r1, r1
    1de4:	4053      	eors	r3, r2
    1de6:	428c      	cmp	r4, r1
    1de8:	d1f0      	bne.n	0x1dcc
    1dea:	0419      	lsls	r1, r3, #16
    1dec:	021e      	lsls	r6, r3, #8
    1dee:	0e09      	lsrs	r1, r1, #24
    1df0:	1859      	adds	r1, r3, r1
    1df2:	0e36      	lsrs	r6, r6, #24
    1df4:	9d01      	ldr	r5, [sp, #4]
    1df6:	1989      	adds	r1, r1, r6
    1df8:	0e1b      	lsrs	r3, r3, #24
    1dfa:	18cb      	adds	r3, r1, r3
    1dfc:	002e      	movs	r6, r5
    1dfe:	2103      	movs	r1, #3
    1e00:	4664      	mov	r4, ip
    1e02:	9802      	ldr	r0, [sp, #8]
    1e04:	9a03      	ldr	r2, [sp, #12]
    1e06:	b2db      	uxtb	r3, r3
    1e08:	438e      	bics	r6, r1
    1e0a:	4229      	tst	r1, r5
    1e0c:	d008      	beq.n	0x1e20
    1e0e:	19a1      	adds	r1, r4, r6
    1e10:	3120      	adds	r1, #32
    1e12:	7809      	ldrb	r1, [r1, #0]
    1e14:	3601      	adds	r6, #1
    1e16:	185b      	adds	r3, r3, r1
    1e18:	b2f6      	uxtb	r6, r6
    1e1a:	b2db      	uxtb	r3, r3
    1e1c:	42b5      	cmp	r5, r6
    1e1e:	d8f6      	bhi.n	0x1e0e
    1e20:	2116      	movs	r1, #22
    1e22:	7003      	strb	r3, [r0, #0]
    1e24:	7813      	ldrb	r3, [r2, #0]
    1e26:	0020      	movs	r0, r4
    1e28:	18e3      	adds	r3, r4, r3
    1e2a:	3321      	adds	r3, #33	@ 0x21
    1e2c:	7019      	strb	r1, [r3, #0]
    1e2e:	7817      	ldrb	r7, [r2, #0]
    1e30:	2380      	movs	r3, #128	@ 0x80
    1e32:	22a0      	movs	r2, #160	@ 0xa0
    1e34:	2100      	movs	r1, #0
    1e36:	1dbd      	adds	r5, r7, #6
    1e38:	05d2      	lsls	r2, r2, #23
    1e3a:	005b      	lsls	r3, r3, #1
    1e3c:	9501      	str	r5, [sp, #4]
    1e3e:	50d1      	str	r1, [r2, r3]
    1e40:	22ff      	movs	r2, #255	@ 0xff
    1e42:	4937      	ldr	r1, [pc, #220]	@ (0x1f20)
    1e44:	402a      	ands	r2, r5
    1e46:	303c      	adds	r0, #60	@ 0x3c
    1e48:	7225      	strb	r5, [r4, #8]
    1e4a:	f001 f829 	bl	0x2ea0
    1e4e:	2d0e      	cmp	r5, #14
    1e50:	dd58      	ble.n	0x1f04
    1e52:	2700      	movs	r7, #0
    1e54:	4d33      	ldr	r5, [pc, #204]	@ (0x1f24)
    1e56:	9402      	str	r4, [sp, #8]
    1e58:	4b31      	ldr	r3, [pc, #196]	@ (0x1f20)
    1e5a:	19dc      	adds	r4, r3, r7
    1e5c:	0026      	movs	r6, r4
    1e5e:	360f      	adds	r6, #15
    1e60:	7821      	ldrb	r1, [r4, #0]
    1e62:	0028      	movs	r0, r5
    1e64:	3401      	adds	r4, #1
    1e66:	f7fe fa01 	bl	0x26c
    1e6a:	42b4      	cmp	r4, r6
    1e6c:	d1f8      	bne.n	0x1e60
    1e6e:	9b01      	ldr	r3, [sp, #4]
    1e70:	370f      	adds	r7, #15
    1e72:	3b0f      	subs	r3, #15
    1e74:	b2ff      	uxtb	r7, r7
    1e76:	2b0e      	cmp	r3, #14
    1e78:	dd01      	ble.n	0x1e7e
    1e7a:	9301      	str	r3, [sp, #4]
    1e7c:	e7ec      	b.n	0x1e58
    1e7e:	9c02      	ldr	r4, [sp, #8]
    1e80:	0026      	movs	r6, r4
    1e82:	361c      	adds	r6, #28
    1e84:	19f6      	adds	r6, r6, r7
    1e86:	9f01      	ldr	r7, [sp, #4]
    1e88:	3f10      	subs	r7, #16
    1e8a:	2b00      	cmp	r3, #0
    1e8c:	d008      	beq.n	0x1ea0
    1e8e:	19f7      	adds	r7, r6, r7
    1e90:	7831      	ldrb	r1, [r6, #0]
    1e92:	0028      	movs	r0, r5
    1e94:	f7fe f9ea 	bl	0x26c
    1e98:	0033      	movs	r3, r6
    1e9a:	3601      	adds	r6, #1
    1e9c:	429f      	cmp	r7, r3
    1e9e:	d1f7      	bne.n	0x1e90
    1ea0:	4b21      	ldr	r3, [pc, #132]	@ (0x1f28)
    1ea2:	681a      	ldr	r2, [r3, #0]
    1ea4:	681b      	ldr	r3, [r3, #0]
    1ea6:	001a      	movs	r2, r3
    1ea8:	3236      	adds	r2, #54	@ 0x36
    1eaa:	3336      	adds	r3, #54	@ 0x36
    1eac:	d100      	bne.n	0x1eb0
    1eae:	2201      	movs	r2, #1
    1eb0:	60e2      	str	r2, [r4, #12]
    1eb2:	b005      	add	sp, #20
    1eb4:	bdf0      	pop	{r4, r5, r6, r7, pc}
    1eb6:	250f      	movs	r5, #15
    1eb8:	7804      	ldrb	r4, [r0, #0]
    1eba:	402c      	ands	r4, r5
    1ebc:	2c03      	cmp	r4, #3
    1ebe:	d000      	beq.n	0x1ec2
    1ec0:	e753      	b.n	0x1d6a
    1ec2:	2723      	movs	r7, #35	@ 0x23
    1ec4:	2524      	movs	r5, #36	@ 0x24
    1ec6:	4c13      	ldr	r4, [pc, #76]	@ (0x1f14)
    1ec8:	5d65      	ldrb	r5, [r4, r5]
    1eca:	5de6      	ldrb	r6, [r4, r7]
    1ecc:	432e      	orrs	r6, r5
    1ece:	2525      	movs	r5, #37	@ 0x25
    1ed0:	5d65      	ldrb	r5, [r4, r5]
    1ed2:	432e      	orrs	r6, r5
    1ed4:	2526      	movs	r5, #38	@ 0x26
    1ed6:	5d65      	ldrb	r5, [r4, r5]
    1ed8:	432e      	orrs	r6, r5
    1eda:	d000      	beq.n	0x1ede
    1edc:	e746      	b.n	0x1d6c
    1ede:	4e13      	ldr	r6, [pc, #76]	@ (0x1f2c)
    1ee0:	8d25      	ldrh	r5, [r4, #40]	@ 0x28
    1ee2:	42b5      	cmp	r5, r6
    1ee4:	d000      	beq.n	0x1ee8
    1ee6:	e741      	b.n	0x1d6c
    1ee8:	2627      	movs	r6, #39	@ 0x27
    1eea:	250f      	movs	r5, #15
    1eec:	5da6      	ldrb	r6, [r4, r6]
    1eee:	4035      	ands	r5, r6
    1ef0:	2d01      	cmp	r5, #1
    1ef2:	d000      	beq.n	0x1ef6
    1ef4:	e73a      	b.n	0x1d6c
    1ef6:	2624      	movs	r6, #36	@ 0x24
    1ef8:	35fe      	adds	r5, #254	@ 0xfe
    1efa:	55e5      	strb	r5, [r4, r7]
    1efc:	55a5      	strb	r5, [r4, r6]
    1efe:	e735      	b.n	0x1d6c
    1f00:	2300      	movs	r3, #0
    1f02:	e78d      	b.n	0x1e20
    1f04:	4e06      	ldr	r6, [pc, #24]	@ (0x1f20)
    1f06:	4d07      	ldr	r5, [pc, #28]	@ (0x1f24)
    1f08:	3705      	adds	r7, #5
    1f0a:	e7c0      	b.n	0x1e8e
    1f0c:	2300      	movs	r3, #0
    1f0e:	2600      	movs	r6, #0
    1f10:	e77d      	b.n	0x1e0e
    1f12:	46c0      	nop			@ (mov r8, r8)
    1f14:	00f0      	lsls	r0, r6, #3
    1f16:	1000      	asrs	r0, r0, #32
    1f18:	7f7f      	ldrb	r7, [r7, #29]
    1f1a:	7f7f      	ldrb	r7, [r7, #29]
    1f1c:	8080      	strh	r0, [r0, #4]
    1f1e:	8080      	strh	r0, [r0, #4]
    1f20:	010c      	lsls	r4, r1, #4
    1f22:	1000      	asrs	r0, r0, #32
    1f24:	0194      	lsls	r4, r2, #6
    1f26:	1000      	asrs	r0, r0, #32
    1f28:	0070      	lsls	r0, r6, #1
    1f2a:	1000      	asrs	r0, r0, #32
    1f2c:	4001      	ands	r1, r0
    1f2e:	0000      	movs	r0, r0
    1f30:	b5f0      	push	{r4, r5, r6, r7, lr}
    1f32:	4fb1      	ldr	r7, [pc, #708]	@ (0x21f8)
    1f34:	4eb1      	ldr	r6, [pc, #708]	@ (0x21fc)
    1f36:	6ffb      	ldr	r3, [r7, #124]	@ 0x7c
    1f38:	b08b      	sub	sp, #44	@ 0x2c
    1f3a:	2b00      	cmp	r3, #0
    1f3c:	d000      	beq.n	0x1f40
    1f3e:	e145      	b.n	0x21cc
    1f40:	238c      	movs	r3, #140	@ 0x8c
    1f42:	4caf      	ldr	r4, [pc, #700]	@ (0x2200)
    1f44:	005b      	lsls	r3, r3, #1
    1f46:	5ce3      	ldrb	r3, [r4, r3]
    1f48:	2b00      	cmp	r3, #0
    1f4a:	d100      	bne.n	0x1f4e
    1f4c:	e0ed      	b.n	0x212a
    1f4e:	4bad      	ldr	r3, [pc, #692]	@ (0x2204)
    1f50:	9601      	str	r6, [sp, #4]
    1f52:	9302      	str	r3, [sp, #8]
    1f54:	9400      	str	r4, [sp, #0]
    1f56:	e018      	b.n	0x1f8a
    1f58:	2da0      	cmp	r5, #160	@ 0xa0
    1f5a:	d100      	bne.n	0x1f5e
    1f5c:	e108      	b.n	0x2170
    1f5e:	d900      	bls.n	0x1f62
    1f60:	e0cc      	b.n	0x20fc
    1f62:	2d10      	cmp	r5, #16
    1f64:	d001      	beq.n	0x1f6a
    1f66:	2d68      	cmp	r5, #104	@ 0x68
    1f68:	d108      	bne.n	0x1f7c
    1f6a:	797b      	ldrb	r3, [r7, #5]
    1f6c:	743d      	strb	r5, [r7, #16]
    1f6e:	2b1f      	cmp	r3, #31
    1f70:	d900      	bls.n	0x1f74
    1f72:	e0fc      	b.n	0x216e
    1f74:	1c5a      	adds	r2, r3, #1
    1f76:	717a      	strb	r2, [r7, #5]
    1f78:	4aa2      	ldr	r2, [pc, #648]	@ (0x2204)
    1f7a:	54d5      	strb	r5, [r2, r3]
    1f7c:	238c      	movs	r3, #140	@ 0x8c
    1f7e:	9a00      	ldr	r2, [sp, #0]
    1f80:	005b      	lsls	r3, r3, #1
    1f82:	5cd3      	ldrb	r3, [r2, r3]
    1f84:	2b00      	cmp	r3, #0
    1f86:	d100      	bne.n	0x1f8a
    1f88:	e0ce      	b.n	0x2128
    1f8a:	217f      	movs	r1, #127	@ 0x7f
    1f8c:	9c00      	ldr	r4, [sp, #0]
    1f8e:	000e      	movs	r6, r1
    1f90:	68e3      	ldr	r3, [r4, #12]
    1f92:	3301      	adds	r3, #1
    1f94:	400b      	ands	r3, r1
    1f96:	68a1      	ldr	r1, [r4, #8]
    1f98:	68e0      	ldr	r0, [r4, #12]
    1f9a:	68a2      	ldr	r2, [r4, #8]
    1f9c:	4290      	cmp	r0, r2
    1f9e:	d100      	bne.n	0x1fa2
    1fa0:	e1a5      	b.n	0x22ee
    1fa2:	68a2      	ldr	r2, [r4, #8]
    1fa4:	18a2      	adds	r2, r4, r2
    1fa6:	7e15      	ldrb	r5, [r2, #24]
    1fa8:	68a2      	ldr	r2, [r4, #8]
    1faa:	3201      	adds	r2, #1
    1fac:	60a2      	str	r2, [r4, #8]
    1fae:	68a2      	ldr	r2, [r4, #8]
    1fb0:	4032      	ands	r2, r6
    1fb2:	60a2      	str	r2, [r4, #8]
    1fb4:	428b      	cmp	r3, r1
    1fb6:	d100      	bne.n	0x1fba
    1fb8:	e0c5      	b.n	0x2146
    1fba:	9b01      	ldr	r3, [sp, #4]
    1fbc:	681b      	ldr	r3, [r3, #0]
    1fbe:	61bb      	str	r3, [r7, #24]
    1fc0:	7c3b      	ldrb	r3, [r7, #16]
    1fc2:	2b00      	cmp	r3, #0
    1fc4:	d0c8      	beq.n	0x1f58
    1fc6:	797a      	ldrb	r2, [r7, #5]
    1fc8:	2a1f      	cmp	r2, #31
    1fca:	d900      	bls.n	0x1fce
    1fcc:	e0cf      	b.n	0x216e
    1fce:	1c51      	adds	r1, r2, #1
    1fd0:	b2cc      	uxtb	r4, r1
    1fd2:	9902      	ldr	r1, [sp, #8]
    1fd4:	717c      	strb	r4, [r7, #5]
    1fd6:	548d      	strb	r5, [r1, r2]
    1fd8:	2d16      	cmp	r5, #22
    1fda:	d1cf      	bne.n	0x1f7c
    1fdc:	2b10      	cmp	r3, #16
    1fde:	d100      	bne.n	0x1fe2
    1fe0:	e0d5      	b.n	0x218e
    1fe2:	2b68      	cmp	r3, #104	@ 0x68
    1fe4:	d1ca      	bne.n	0x1f7c
    1fe6:	2c05      	cmp	r4, #5
    1fe8:	d9c8      	bls.n	0x1f7c
    1fea:	780b      	ldrb	r3, [r1, #0]
    1fec:	000a      	movs	r2, r1
    1fee:	2b68      	cmp	r3, #104	@ 0x68
    1ff0:	d1c4      	bne.n	0x1f7c
    1ff2:	78cb      	ldrb	r3, [r1, #3]
    1ff4:	2b68      	cmp	r3, #104	@ 0x68
    1ff6:	d1c1      	bne.n	0x1f7c
    1ff8:	7849      	ldrb	r1, [r1, #1]
    1ffa:	7893      	ldrb	r3, [r2, #2]
    1ffc:	9103      	str	r1, [sp, #12]
    1ffe:	428b      	cmp	r3, r1
    2000:	d1bc      	bne.n	0x1f7c
    2002:	1913      	adds	r3, r2, r4
    2004:	3b01      	subs	r3, #1
    2006:	781b      	ldrb	r3, [r3, #0]
    2008:	2b16      	cmp	r3, #22
    200a:	d1b7      	bne.n	0x1f7c
    200c:	1d8b      	adds	r3, r1, #6
    200e:	429c      	cmp	r4, r3
    2010:	d1b4      	bne.n	0x1f7c
    2012:	2900      	cmp	r1, #0
    2014:	d100      	bne.n	0x2018
    2016:	e30d      	b.n	0x2634
    2018:	1e4b      	subs	r3, r1, #1
    201a:	2b15      	cmp	r3, #21
    201c:	d800      	bhi.n	0x2020
    201e:	e303      	b.n	0x2628
    2020:	1d15      	adds	r5, r2, #4
    2022:	4a79      	ldr	r2, [pc, #484]	@ (0x2208)
    2024:	088b      	lsrs	r3, r1, #2
    2026:	9304      	str	r3, [sp, #16]
    2028:	2100      	movs	r1, #0
    202a:	2300      	movs	r3, #0
    202c:	4694      	mov	ip, r2
    202e:	9e01      	ldr	r6, [sp, #4]
    2030:	9405      	str	r4, [sp, #20]
    2032:	4660      	mov	r0, ip
    2034:	4664      	mov	r4, ip
    2036:	cd04      	ldmia	r5!, {r2}
    2038:	401c      	ands	r4, r3
    203a:	4010      	ands	r0, r2
    203c:	405a      	eors	r2, r3
    203e:	4b73      	ldr	r3, [pc, #460]	@ (0x220c)
    2040:	1900      	adds	r0, r0, r4
    2042:	401a      	ands	r2, r3
    2044:	0003      	movs	r3, r0
    2046:	3101      	adds	r1, #1
    2048:	4053      	eors	r3, r2
    204a:	9a04      	ldr	r2, [sp, #16]
    204c:	b2c9      	uxtb	r1, r1
    204e:	428a      	cmp	r2, r1
    2050:	d1ef      	bne.n	0x2032
    2052:	041a      	lsls	r2, r3, #16
    2054:	0219      	lsls	r1, r3, #8
    2056:	0e12      	lsrs	r2, r2, #24
    2058:	189a      	adds	r2, r3, r2
    205a:	0e09      	lsrs	r1, r1, #24
    205c:	9803      	ldr	r0, [sp, #12]
    205e:	1852      	adds	r2, r2, r1
    2060:	0e1b      	lsrs	r3, r3, #24
    2062:	18d3      	adds	r3, r2, r3
    2064:	0001      	movs	r1, r0
    2066:	2203      	movs	r2, #3
    2068:	9c05      	ldr	r4, [sp, #20]
    206a:	9601      	str	r6, [sp, #4]
    206c:	b2db      	uxtb	r3, r3
    206e:	4391      	bics	r1, r2
    2070:	4202      	tst	r2, r0
    2072:	d00a      	beq.n	0x208a
    2074:	9d02      	ldr	r5, [sp, #8]
    2076:	1d0a      	adds	r2, r1, #4
    2078:	b2d2      	uxtb	r2, r2
    207a:	5caa      	ldrb	r2, [r5, r2]
    207c:	3101      	adds	r1, #1
    207e:	189b      	adds	r3, r3, r2
    2080:	b2c9      	uxtb	r1, r1
    2082:	b2db      	uxtb	r3, r3
    2084:	4288      	cmp	r0, r1
    2086:	d8f6      	bhi.n	0x2076
    2088:	9601      	str	r6, [sp, #4]
    208a:	9902      	ldr	r1, [sp, #8]
    208c:	190a      	adds	r2, r1, r4
    208e:	3a02      	subs	r2, #2
    2090:	7812      	ldrb	r2, [r2, #0]
    2092:	429a      	cmp	r2, r3
    2094:	d000      	beq.n	0x2098
    2096:	e771      	b.n	0x1f7c
    2098:	250f      	movs	r5, #15
    209a:	790b      	ldrb	r3, [r1, #4]
    209c:	401d      	ands	r5, r3
    209e:	2d03      	cmp	r5, #3
    20a0:	d000      	beq.n	0x20a4
    20a2:	e08f      	b.n	0x21c4
    20a4:	794b      	ldrb	r3, [r1, #5]
    20a6:	2b43      	cmp	r3, #67	@ 0x43
    20a8:	d100      	bne.n	0x20ac
    20aa:	e139      	b.n	0x2320
    20ac:	d900      	bls.n	0x20b0
    20ae:	e109      	b.n	0x22c4
    20b0:	2b11      	cmp	r3, #17
    20b2:	d100      	bne.n	0x20b6
    20b4:	e151      	b.n	0x235a
    20b6:	2b41      	cmp	r3, #65	@ 0x41
    20b8:	d000      	beq.n	0x20bc
    20ba:	e083      	b.n	0x21c4
    20bc:	2300      	movs	r3, #0
    20be:	847b      	strh	r3, [r7, #34]	@ 0x22
    20c0:	84bb      	strh	r3, [r7, #36]	@ 0x24
    20c2:	9b02      	ldr	r3, [sp, #8]
    20c4:	899d      	ldrh	r5, [r3, #12]
    20c6:	2d03      	cmp	r5, #3
    20c8:	d000      	beq.n	0x20cc
    20ca:	e07b      	b.n	0x21c4
    20cc:	f7fe f8f4 	bl	0x2b8
    20d0:	2263      	movs	r2, #99	@ 0x63
    20d2:	2327      	movs	r3, #39	@ 0x27
    20d4:	54fa      	strb	r2, [r7, r3]
    20d6:	4b4e      	ldr	r3, [pc, #312]	@ (0x2210)
    20d8:	a809      	add	r0, sp, #36	@ 0x24
    20da:	62bb      	str	r3, [r7, #40]	@ 0x28
    20dc:	ab08      	add	r3, sp, #32
    20de:	1d59      	adds	r1, r3, #5
    20e0:	238e      	movs	r3, #142	@ 0x8e
    20e2:	700b      	strb	r3, [r1, #0]
    20e4:	ab08      	add	r3, sp, #32
    20e6:	1d9a      	adds	r2, r3, #6
    20e8:	230c      	movs	r3, #12
    20ea:	7005      	strb	r5, [r0, #0]
    20ec:	2501      	movs	r5, #1
    20ee:	7013      	strb	r3, [r2, #0]
    20f0:	ab08      	add	r3, sp, #32
    20f2:	3307      	adds	r3, #7
    20f4:	701d      	strb	r5, [r3, #0]
    20f6:	f7ff fe2f 	bl	0x1d58
    20fa:	e063      	b.n	0x21c4
    20fc:	2de5      	cmp	r5, #229	@ 0xe5
    20fe:	d000      	beq.n	0x2102
    2100:	e73c      	b.n	0x1f7c
    2102:	2201      	movs	r2, #1
    2104:	60fb      	str	r3, [r7, #12]
    2106:	723b      	strb	r3, [r7, #8]
    2108:	793b      	ldrb	r3, [r7, #4]
    210a:	2140      	movs	r1, #64	@ 0x40
    210c:	4053      	eors	r3, r2
    210e:	713b      	strb	r3, [r7, #4]
    2110:	22a0      	movs	r2, #160	@ 0xa0
    2112:	2380      	movs	r3, #128	@ 0x80
    2114:	05d2      	lsls	r2, r2, #23
    2116:	005b      	lsls	r3, r3, #1
    2118:	50d1      	str	r1, [r2, r3]
    211a:	238c      	movs	r3, #140	@ 0x8c
    211c:	9a00      	ldr	r2, [sp, #0]
    211e:	005b      	lsls	r3, r3, #1
    2120:	5cd3      	ldrb	r3, [r2, r3]
    2122:	2b00      	cmp	r3, #0
    2124:	d000      	beq.n	0x2128
    2126:	e730      	b.n	0x1f8a
    2128:	9e01      	ldr	r6, [sp, #4]
    212a:	68fb      	ldr	r3, [r7, #12]
    212c:	2b00      	cmp	r3, #0
    212e:	d000      	beq.n	0x2132
    2130:	e0ab      	b.n	0x228a
    2132:	6b74      	ldr	r4, [r6, #52]	@ 0x34
    2134:	6863      	ldr	r3, [r4, #4]
    2136:	2b00      	cmp	r3, #0
    2138:	d000      	beq.n	0x213c
    213a:	e06f      	b.n	0x221c
    213c:	7c3b      	ldrb	r3, [r7, #16]
    213e:	2b00      	cmp	r3, #0
    2140:	d151      	bne.n	0x21e6
    2142:	b00b      	add	sp, #44	@ 0x2c
    2144:	bdf0      	pop	{r4, r5, r6, r7, pc}
    2146:	4b33      	ldr	r3, [pc, #204]	@ (0x2214)
    2148:	695b      	ldr	r3, [r3, #20]
    214a:	07db      	lsls	r3, r3, #31
    214c:	d400      	bmi.n	0x2150
    214e:	e734      	b.n	0x1fba
    2150:	f3bf 8f4f 	dsb	sy
    2154:	f3bf 8f6f 	isb	sy
    2158:	4a2f      	ldr	r2, [pc, #188]	@ (0x2218)
    215a:	2480      	movs	r4, #128	@ 0x80
    215c:	2380      	movs	r3, #128	@ 0x80
    215e:	0016      	movs	r6, r2
    2160:	03a4      	lsls	r4, r4, #14
    2162:	9800      	ldr	r0, [sp, #0]
    2164:	50d4      	str	r4, [r2, r3]
    2166:	f7fe f9f1 	bl	0x54c
    216a:	6034      	str	r4, [r6, #0]
    216c:	e725      	b.n	0x1fba
    216e:	2300      	movs	r3, #0
    2170:	9a00      	ldr	r2, [sp, #0]
    2172:	6093      	str	r3, [r2, #8]
    2174:	60d3      	str	r3, [r2, #12]
    2176:	6113      	str	r3, [r2, #16]
    2178:	6153      	str	r3, [r2, #20]
    217a:	2201      	movs	r2, #1
    217c:	713a      	strb	r2, [r7, #4]
    217e:	2200      	movs	r2, #0
    2180:	717b      	strb	r3, [r7, #5]
    2182:	813b      	strh	r3, [r7, #8]
    2184:	60fb      	str	r3, [r7, #12]
    2186:	743a      	strb	r2, [r7, #16]
    2188:	617b      	str	r3, [r7, #20]
    218a:	61bb      	str	r3, [r7, #24]
    218c:	e6f6      	b.n	0x1f7c
    218e:	2c04      	cmp	r4, #4
    2190:	d000      	beq.n	0x2194
    2192:	e6f3      	b.n	0x1f7c
    2194:	780b      	ldrb	r3, [r1, #0]
    2196:	2b10      	cmp	r3, #16
    2198:	d000      	beq.n	0x219c
    219a:	e6ef      	b.n	0x1f7c
    219c:	78cb      	ldrb	r3, [r1, #3]
    219e:	2b16      	cmp	r3, #22
    21a0:	d000      	beq.n	0x21a4
    21a2:	e6eb      	b.n	0x1f7c
    21a4:	784a      	ldrb	r2, [r1, #1]
    21a6:	788b      	ldrb	r3, [r1, #2]
    21a8:	4293      	cmp	r3, r2
    21aa:	d000      	beq.n	0x21ae
    21ac:	e6e6      	b.n	0x1f7c
    21ae:	065a      	lsls	r2, r3, #25
    21b0:	d508      	bpl.n	0x21c4
    21b2:	220f      	movs	r2, #15
    21b4:	0011      	movs	r1, r2
    21b6:	4019      	ands	r1, r3
    21b8:	421a      	tst	r2, r3
    21ba:	d074      	beq.n	0x22a6
    21bc:	2909      	cmp	r1, #9
    21be:	d101      	bne.n	0x21c4
    21c0:	f7fe f87a 	bl	0x2b8
    21c4:	2300      	movs	r3, #0
    21c6:	743b      	strb	r3, [r7, #16]
    21c8:	717b      	strb	r3, [r7, #5]
    21ca:	e6d7      	b.n	0x1f7c
    21cc:	6832      	ldr	r2, [r6, #0]
    21ce:	1ad3      	subs	r3, r2, r3
    21d0:	d500      	bpl.n	0x21d4
    21d2:	e6b5      	b.n	0x1f40
    21d4:	2300      	movs	r3, #0
    21d6:	22a0      	movs	r2, #160	@ 0xa0
    21d8:	67fb      	str	r3, [r7, #124]	@ 0x7c
    21da:	2380      	movs	r3, #128	@ 0x80
    21dc:	2180      	movs	r1, #128	@ 0x80
    21de:	05d2      	lsls	r2, r2, #23
    21e0:	009b      	lsls	r3, r3, #2
    21e2:	50d1      	str	r1, [r2, r3]
    21e4:	e6ac      	b.n	0x1f40
    21e6:	6833      	ldr	r3, [r6, #0]
    21e8:	69ba      	ldr	r2, [r7, #24]
    21ea:	1a9b      	subs	r3, r3, r2
    21ec:	2b36      	cmp	r3, #54	@ 0x36
    21ee:	d9a8      	bls.n	0x2142
    21f0:	2300      	movs	r3, #0
    21f2:	743b      	strb	r3, [r7, #16]
    21f4:	717b      	strb	r3, [r7, #5]
    21f6:	e7a4      	b.n	0x2142
    21f8:	00f0      	lsls	r0, r6, #3
    21fa:	1000      	asrs	r0, r0, #32
    21fc:	0070      	lsls	r0, r6, #1
    21fe:	1000      	asrs	r0, r0, #32
    2200:	0194      	lsls	r4, r2, #6
    2202:	1000      	asrs	r0, r0, #32
    2204:	0170      	lsls	r0, r6, #5
    2206:	1000      	asrs	r0, r0, #32
    2208:	7f7f      	ldrb	r7, [r7, #29]
    220a:	7f7f      	ldrb	r7, [r7, #29]
    220c:	8080      	strh	r0, [r0, #4]
    220e:	8080      	strh	r0, [r0, #4]
    2210:	4003      	ands	r3, r0
    2212:	1200      	asrs	r0, r0, #8
    2214:	8000      	strh	r0, [r0, #0]
    2216:	4000      	ands	r0, r0
    2218:	e100      	b.n	0x241c
    221a:	e000      	b.n	0x221e
    221c:	22a0      	movs	r2, #160	@ 0xa0
    221e:	2380      	movs	r3, #128	@ 0x80
    2220:	2100      	movs	r1, #0
    2222:	05d2      	lsls	r2, r2, #23
    2224:	009b      	lsls	r3, r3, #2
    2226:	50d1      	str	r1, [r2, r3]
    2228:	6833      	ldr	r3, [r6, #0]
    222a:	001a      	movs	r2, r3
    222c:	3264      	adds	r2, #100	@ 0x64
    222e:	3364      	adds	r3, #100	@ 0x64
    2230:	d032      	beq.n	0x2298
    2232:	7a3b      	ldrb	r3, [r7, #8]
    2234:	67fa      	str	r2, [r7, #124]	@ 0x7c
    2236:	2b00      	cmp	r3, #0
    2238:	d000      	beq.n	0x223c
    223a:	e77f      	b.n	0x213c
    223c:	6862      	ldr	r2, [r4, #4]
    223e:	2a01      	cmp	r2, #1
    2240:	dd0c      	ble.n	0x225c
    2242:	2200      	movs	r2, #0
    2244:	6821      	ldr	r1, [r4, #0]
    2246:	18ba      	adds	r2, r7, r2
    2248:	5cc9      	ldrb	r1, [r1, r3]
    224a:	3222      	adds	r2, #34	@ 0x22
    224c:	7011      	strb	r1, [r2, #0]
    224e:	6861      	ldr	r1, [r4, #4]
    2250:	3301      	adds	r3, #1
    2252:	b2db      	uxtb	r3, r3
    2254:	3901      	subs	r1, #1
    2256:	001a      	movs	r2, r3
    2258:	428b      	cmp	r3, r1
    225a:	dbf3      	blt.n	0x2244
    225c:	2320      	movs	r3, #32
    225e:	22f0      	movs	r2, #240	@ 0xf0
    2260:	54fa      	strb	r2, [r7, r3]
    2262:	ab08      	add	r3, sp, #32
    2264:	1dd8      	adds	r0, r3, #7
    2266:	2303      	movs	r3, #3
    2268:	7003      	strb	r3, [r0, #0]
    226a:	ab08      	add	r3, sp, #32
    226c:	1d99      	adds	r1, r3, #6
    226e:	2329      	movs	r3, #41	@ 0x29
    2270:	700b      	strb	r3, [r1, #0]
    2272:	6863      	ldr	r3, [r4, #4]
    2274:	aa08      	add	r2, sp, #32
    2276:	3205      	adds	r2, #5
    2278:	3301      	adds	r3, #1
    227a:	7013      	strb	r3, [r2, #0]
    227c:	1d3b      	adds	r3, r7, #4
    227e:	f7ff fd6b 	bl	0x1d58
    2282:	2200      	movs	r2, #0
    2284:	6b73      	ldr	r3, [r6, #52]	@ 0x34
    2286:	605a      	str	r2, [r3, #4]
    2288:	e758      	b.n	0x213c
    228a:	6832      	ldr	r2, [r6, #0]
    228c:	1ad3      	subs	r3, r2, r3
    228e:	d500      	bpl.n	0x2292
    2290:	e74f      	b.n	0x2132
    2292:	2300      	movs	r3, #0
    2294:	60fb      	str	r3, [r7, #12]
    2296:	e74c      	b.n	0x2132
    2298:	2201      	movs	r2, #1
    229a:	7a3b      	ldrb	r3, [r7, #8]
    229c:	67fa      	str	r2, [r7, #124]	@ 0x7c
    229e:	2b00      	cmp	r3, #0
    22a0:	d000      	beq.n	0x22a4
    22a2:	e74b      	b.n	0x213c
    22a4:	e7ca      	b.n	0x223c
    22a6:	f7fe f807 	bl	0x2b8
    22aa:	2300      	movs	r3, #0
    22ac:	9a00      	ldr	r2, [sp, #0]
    22ae:	813b      	strh	r3, [r7, #8]
    22b0:	6093      	str	r3, [r2, #8]
    22b2:	60d3      	str	r3, [r2, #12]
    22b4:	6113      	str	r3, [r2, #16]
    22b6:	6153      	str	r3, [r2, #20]
    22b8:	2201      	movs	r2, #1
    22ba:	60fb      	str	r3, [r7, #12]
    22bc:	713a      	strb	r2, [r7, #4]
    22be:	617b      	str	r3, [r7, #20]
    22c0:	61bb      	str	r3, [r7, #24]
    22c2:	e77f      	b.n	0x21c4
    22c4:	2ba7      	cmp	r3, #167	@ 0xa7
    22c6:	d100      	bne.n	0x22ca
    22c8:	e168      	b.n	0x259c
    22ca:	2ba9      	cmp	r3, #169	@ 0xa9
    22cc:	d000      	beq.n	0x22d0
    22ce:	e779      	b.n	0x21c4
    22d0:	2300      	movs	r3, #0
    22d2:	9a00      	ldr	r2, [sp, #0]
    22d4:	813b      	strh	r3, [r7, #8]
    22d6:	6093      	str	r3, [r2, #8]
    22d8:	60d3      	str	r3, [r2, #12]
    22da:	6113      	str	r3, [r2, #16]
    22dc:	6153      	str	r3, [r2, #20]
    22de:	2201      	movs	r2, #1
    22e0:	60fb      	str	r3, [r7, #12]
    22e2:	713a      	strb	r2, [r7, #4]
    22e4:	617b      	str	r3, [r7, #20]
    22e6:	61bb      	str	r3, [r7, #24]
    22e8:	f7fd ffe6 	bl	0x2b8
    22ec:	e76a      	b.n	0x21c4
    22ee:	9e01      	ldr	r6, [sp, #4]
    22f0:	428b      	cmp	r3, r1
    22f2:	d000      	beq.n	0x22f6
    22f4:	e719      	b.n	0x212a
    22f6:	4bd0      	ldr	r3, [pc, #832]	@ (0x2638)
    22f8:	695b      	ldr	r3, [r3, #20]
    22fa:	07db      	lsls	r3, r3, #31
    22fc:	d400      	bmi.n	0x2300
    22fe:	e714      	b.n	0x212a
    2300:	f3bf 8f4f 	dsb	sy
    2304:	f3bf 8f6f 	isb	sy
    2308:	2280      	movs	r2, #128	@ 0x80
    230a:	2380      	movs	r3, #128	@ 0x80
    230c:	4dcb      	ldr	r5, [pc, #812]	@ (0x263c)
    230e:	0392      	lsls	r2, r2, #14
    2310:	50ea      	str	r2, [r5, r3]
    2312:	0020      	movs	r0, r4
    2314:	f7fe f91a 	bl	0x54c
    2318:	2280      	movs	r2, #128	@ 0x80
    231a:	0392      	lsls	r2, r2, #14
    231c:	602a      	str	r2, [r5, #0]
    231e:	e704      	b.n	0x212a
    2320:	f7fd ffca 	bl	0x2b8
    2324:	2300      	movs	r3, #0
    2326:	2222      	movs	r2, #34	@ 0x22
    2328:	9802      	ldr	r0, [sp, #8]
    232a:	54bb      	strb	r3, [r7, r2]
    232c:	7a41      	ldrb	r1, [r0, #9]
    232e:	3201      	adds	r2, #1
    2330:	54b9      	strb	r1, [r7, r2]
    2332:	7a81      	ldrb	r1, [r0, #10]
    2334:	3201      	adds	r2, #1
    2336:	54b9      	strb	r1, [r7, r2]
    2338:	3201      	adds	r2, #1
    233a:	54bb      	strb	r3, [r7, r2]
    233c:	84fb      	strh	r3, [r7, #38]	@ 0x26
    233e:	ab08      	add	r3, sp, #32
    2340:	1dd8      	adds	r0, r3, #7
    2342:	1d99      	adds	r1, r3, #6
    2344:	2386      	movs	r3, #134	@ 0x86
    2346:	700b      	strb	r3, [r1, #0]
    2348:	ab08      	add	r3, sp, #32
    234a:	1d5a      	adds	r2, r3, #5
    234c:	2307      	movs	r3, #7
    234e:	7005      	strb	r5, [r0, #0]
    2350:	7013      	strb	r3, [r2, #0]
    2352:	1d3b      	adds	r3, r7, #4
    2354:	f7ff fd00 	bl	0x1d58
    2358:	e734      	b.n	0x21c4
    235a:	f7fd ffad 	bl	0x2b8
    235e:	9b02      	ldr	r3, [sp, #8]
    2360:	aa08      	add	r2, sp, #32
    2362:	785b      	ldrb	r3, [r3, #1]
    2364:	9304      	str	r3, [sp, #16]
    2366:	7153      	strb	r3, [r2, #5]
    2368:	697b      	ldr	r3, [r7, #20]
    236a:	2b00      	cmp	r3, #0
    236c:	d100      	bne.n	0x2370
    236e:	e134      	b.n	0x25da
    2370:	9a01      	ldr	r2, [sp, #4]
    2372:	6fd5      	ldr	r5, [r2, #124]	@ 0x7c
    2374:	425a      	negs	r2, r3
    2376:	4153      	adcs	r3, r2
    2378:	9a04      	ldr	r2, [sp, #16]
    237a:	617b      	str	r3, [r7, #20]
    237c:	1e93      	subs	r3, r2, #2
    237e:	2a05      	cmp	r2, #5
    2380:	d909      	bls.n	0x2396
    2382:	2203      	movs	r2, #3
    2384:	9e01      	ldr	r6, [sp, #4]
    2386:	9802      	ldr	r0, [sp, #8]
    2388:	1881      	adds	r1, r0, r2
    238a:	7989      	ldrb	r1, [r1, #6]
    238c:	54a9      	strb	r1, [r5, r2]
    238e:	3201      	adds	r2, #1
    2390:	429a      	cmp	r2, r3
    2392:	dbf9      	blt.n	0x2388
    2394:	9601      	str	r6, [sp, #4]
    2396:	9a02      	ldr	r2, [sp, #8]
    2398:	210f      	movs	r1, #15
    239a:	7990      	ldrb	r0, [r2, #6]
    239c:	2250      	movs	r2, #80	@ 0x50
    239e:	4001      	ands	r1, r0
    23a0:	4252      	negs	r2, r2
    23a2:	430a      	orrs	r2, r1
    23a4:	702a      	strb	r2, [r5, #0]
    23a6:	220c      	movs	r2, #12
    23a8:	2122      	movs	r1, #34	@ 0x22
    23aa:	4002      	ands	r2, r0
    23ac:	547a      	strb	r2, [r7, r1]
    23ae:	2c09      	cmp	r4, #9
    23b0:	d946      	bls.n	0x2440
    23b2:	9903      	ldr	r1, [sp, #12]
    23b4:	2201      	movs	r2, #1
    23b6:	2903      	cmp	r1, #3
    23b8:	d900      	bls.n	0x23bc
    23ba:	1eca      	subs	r2, r1, #3
    23bc:	9903      	ldr	r1, [sp, #12]
    23be:	2907      	cmp	r1, #7
    23c0:	d800      	bhi.n	0x23c4
    23c2:	e12e      	b.n	0x2622
    23c4:	2123      	movs	r1, #35	@ 0x23
    23c6:	9e02      	ldr	r6, [sp, #8]
    23c8:	1e54      	subs	r4, r2, #1
    23ca:	79f0      	ldrb	r0, [r6, #7]
    23cc:	08a2      	lsrs	r2, r4, #2
    23ce:	5478      	strb	r0, [r7, r1]
    23d0:	68b1      	ldr	r1, [r6, #8]
    23d2:	6279      	str	r1, [r7, #36]	@ 0x24
    23d4:	2a01      	cmp	r2, #1
    23d6:	d011      	beq.n	0x23fc
    23d8:	68f1      	ldr	r1, [r6, #12]
    23da:	62b9      	str	r1, [r7, #40]	@ 0x28
    23dc:	2a02      	cmp	r2, #2
    23de:	d00d      	beq.n	0x23fc
    23e0:	6931      	ldr	r1, [r6, #16]
    23e2:	62f9      	str	r1, [r7, #44]	@ 0x2c
    23e4:	2a03      	cmp	r2, #3
    23e6:	d009      	beq.n	0x23fc
    23e8:	6971      	ldr	r1, [r6, #20]
    23ea:	6339      	str	r1, [r7, #48]	@ 0x30
    23ec:	2a04      	cmp	r2, #4
    23ee:	d005      	beq.n	0x23fc
    23f0:	69b1      	ldr	r1, [r6, #24]
    23f2:	6379      	str	r1, [r7, #52]	@ 0x34
    23f4:	2a05      	cmp	r2, #5
    23f6:	d001      	beq.n	0x23fc
    23f8:	69f2      	ldr	r2, [r6, #28]
    23fa:	63ba      	str	r2, [r7, #56]	@ 0x38
    23fc:	2003      	movs	r0, #3
    23fe:	0021      	movs	r1, r4
    2400:	4381      	bics	r1, r0
    2402:	3108      	adds	r1, #8
    2404:	b2ca      	uxtb	r2, r1
    2406:	4220      	tst	r0, r4
    2408:	d01a      	beq.n	0x2440
    240a:	9e02      	ldr	r6, [sp, #8]
    240c:	1878      	adds	r0, r7, r1
    240e:	5c71      	ldrb	r1, [r6, r1]
    2410:	9c03      	ldr	r4, [sp, #12]
    2412:	7701      	strb	r1, [r0, #28]
    2414:	1c51      	adds	r1, r2, #1
    2416:	3404      	adds	r4, #4
    2418:	b2c9      	uxtb	r1, r1
    241a:	428c      	cmp	r4, r1
    241c:	dd10      	ble.n	0x2440
    241e:	1878      	adds	r0, r7, r1
    2420:	5c71      	ldrb	r1, [r6, r1]
    2422:	7701      	strb	r1, [r0, #28]
    2424:	1c91      	adds	r1, r2, #2
    2426:	b2c9      	uxtb	r1, r1
    2428:	428c      	cmp	r4, r1
    242a:	dd09      	ble.n	0x2440
    242c:	1878      	adds	r0, r7, r1
    242e:	3203      	adds	r2, #3
    2430:	5c71      	ldrb	r1, [r6, r1]
    2432:	b2d2      	uxtb	r2, r2
    2434:	7701      	strb	r1, [r0, #28]
    2436:	4294      	cmp	r4, r2
    2438:	dd02      	ble.n	0x2440
    243a:	18b9      	adds	r1, r7, r2
    243c:	5cb2      	ldrb	r2, [r6, r2]
    243e:	770a      	strb	r2, [r1, #28]
    2440:	9a01      	ldr	r2, [sp, #4]
    2442:	b298      	uxth	r0, r3
    2444:	6b52      	ldr	r2, [r2, #52]	@ 0x34
    2446:	6893      	ldr	r3, [r2, #8]
    2448:	4694      	mov	ip, r2
    244a:	8a9b      	ldrh	r3, [r3, #20]
    244c:	0a19      	lsrs	r1, r3, #8
    244e:	b2dc      	uxtb	r4, r3
    2450:	7069      	strb	r1, [r5, #1]
    2452:	70ac      	strb	r4, [r5, #2]
    2454:	2800      	cmp	r0, #0
    2456:	d100      	bne.n	0x245a
    2458:	e0e1      	b.n	0x261e
    245a:	2203      	movs	r2, #3
    245c:	426b      	negs	r3, r5
    245e:	b29e      	uxth	r6, r3
    2460:	401a      	ands	r2, r3
    2462:	9b04      	ldr	r3, [sp, #16]
    2464:	3b03      	subs	r3, #3
    2466:	2b0a      	cmp	r3, #10
    2468:	d800      	bhi.n	0x246c
    246a:	e0d5      	b.n	0x2618
    246c:	2a00      	cmp	r2, #0
    246e:	d100      	bne.n	0x2472
    2470:	e0ca      	b.n	0x2608
    2472:	782b      	ldrb	r3, [r5, #0]
    2474:	43db      	mvns	r3, r3
    2476:	b2db      	uxtb	r3, r3
    2478:	9304      	str	r3, [sp, #16]
    247a:	07b3      	lsls	r3, r6, #30
    247c:	d400      	bmi.n	0x2480
    247e:	e0ae      	b.n	0x25de
    2480:	9b04      	ldr	r3, [sp, #16]
    2482:	404b      	eors	r3, r1
    2484:	9304      	str	r3, [sp, #16]
    2486:	2a03      	cmp	r2, #3
    2488:	d000      	beq.n	0x248c
    248a:	e0c2      	b.n	0x2612
    248c:	4063      	eors	r3, r4
    248e:	9304      	str	r3, [sp, #16]
    2490:	2303      	movs	r3, #3
    2492:	9305      	str	r3, [sp, #20]
    2494:	1a83      	subs	r3, r0, r2
    2496:	b29b      	uxth	r3, r3
    2498:	9306      	str	r3, [sp, #24]
    249a:	089b      	lsrs	r3, r3, #2
    249c:	9303      	str	r3, [sp, #12]
    249e:	18ac      	adds	r4, r5, r2
    24a0:	2300      	movs	r3, #0
    24a2:	2200      	movs	r2, #0
    24a4:	9e01      	ldr	r6, [sp, #4]
    24a6:	9007      	str	r0, [sp, #28]
    24a8:	0099      	lsls	r1, r3, #2
    24aa:	5861      	ldr	r1, [r4, r1]
    24ac:	3301      	adds	r3, #1
    24ae:	9803      	ldr	r0, [sp, #12]
    24b0:	404a      	eors	r2, r1
    24b2:	b299      	uxth	r1, r3
    24b4:	4288      	cmp	r0, r1
    24b6:	d8f7      	bhi.n	0x24a8
    24b8:	0411      	lsls	r1, r2, #16
    24ba:	9b04      	ldr	r3, [sp, #16]
    24bc:	0e09      	lsrs	r1, r1, #24
    24be:	4051      	eors	r1, r2
    24c0:	404b      	eors	r3, r1
    24c2:	0019      	movs	r1, r3
    24c4:	0213      	lsls	r3, r2, #8
    24c6:	9c06      	ldr	r4, [sp, #24]
    24c8:	0e1b      	lsrs	r3, r3, #24
    24ca:	404b      	eors	r3, r1
    24cc:	0e12      	lsrs	r2, r2, #24
    24ce:	4053      	eors	r3, r2
    24d0:	2103      	movs	r1, #3
    24d2:	0022      	movs	r2, r4
    24d4:	9601      	str	r6, [sp, #4]
    24d6:	9e05      	ldr	r6, [sp, #20]
    24d8:	438a      	bics	r2, r1
    24da:	18b2      	adds	r2, r6, r2
    24dc:	9807      	ldr	r0, [sp, #28]
    24de:	b2db      	uxtb	r3, r3
    24e0:	b292      	uxth	r2, r2
    24e2:	4221      	tst	r1, r4
    24e4:	d040      	beq.n	0x2568
    24e6:	5ca9      	ldrb	r1, [r5, r2]
    24e8:	404b      	eors	r3, r1
    24ea:	1c51      	adds	r1, r2, #1
    24ec:	b289      	uxth	r1, r1
    24ee:	4281      	cmp	r1, r0
    24f0:	d23a      	bcs.n	0x2568
    24f2:	5c69      	ldrb	r1, [r5, r1]
    24f4:	404b      	eors	r3, r1
    24f6:	1c91      	adds	r1, r2, #2
    24f8:	b289      	uxth	r1, r1
    24fa:	b2db      	uxtb	r3, r3
    24fc:	4288      	cmp	r0, r1
    24fe:	d933      	bls.n	0x2568
    2500:	5c69      	ldrb	r1, [r5, r1]
    2502:	404b      	eors	r3, r1
    2504:	1cd1      	adds	r1, r2, #3
    2506:	b289      	uxth	r1, r1
    2508:	4281      	cmp	r1, r0
    250a:	d22d      	bcs.n	0x2568
    250c:	5c69      	ldrb	r1, [r5, r1]
    250e:	404b      	eors	r3, r1
    2510:	1d11      	adds	r1, r2, #4
    2512:	b289      	uxth	r1, r1
    2514:	4281      	cmp	r1, r0
    2516:	d227      	bcs.n	0x2568
    2518:	5c69      	ldrb	r1, [r5, r1]
    251a:	404b      	eors	r3, r1
    251c:	1d51      	adds	r1, r2, #5
    251e:	b289      	uxth	r1, r1
    2520:	4288      	cmp	r0, r1
    2522:	d921      	bls.n	0x2568
    2524:	5c69      	ldrb	r1, [r5, r1]
    2526:	404b      	eors	r3, r1
    2528:	1d91      	adds	r1, r2, #6
    252a:	b289      	uxth	r1, r1
    252c:	4288      	cmp	r0, r1
    252e:	d91b      	bls.n	0x2568
    2530:	5c69      	ldrb	r1, [r5, r1]
    2532:	404b      	eors	r3, r1
    2534:	1dd1      	adds	r1, r2, #7
    2536:	b289      	uxth	r1, r1
    2538:	4288      	cmp	r0, r1
    253a:	d915      	bls.n	0x2568
    253c:	5c69      	ldrb	r1, [r5, r1]
    253e:	404b      	eors	r3, r1
    2540:	0011      	movs	r1, r2
    2542:	3108      	adds	r1, #8
    2544:	b289      	uxth	r1, r1
    2546:	4288      	cmp	r0, r1
    2548:	d90e      	bls.n	0x2568
    254a:	5c69      	ldrb	r1, [r5, r1]
    254c:	404b      	eors	r3, r1
    254e:	0011      	movs	r1, r2
    2550:	3109      	adds	r1, #9
    2552:	b289      	uxth	r1, r1
    2554:	4288      	cmp	r0, r1
    2556:	d907      	bls.n	0x2568
    2558:	5c69      	ldrb	r1, [r5, r1]
    255a:	320a      	adds	r2, #10
    255c:	b292      	uxth	r2, r2
    255e:	404b      	eors	r3, r1
    2560:	4290      	cmp	r0, r2
    2562:	d901      	bls.n	0x2568
    2564:	5caa      	ldrb	r2, [r5, r2]
    2566:	4053      	eors	r3, r2
    2568:	542b      	strb	r3, [r5, r0]
    256a:	4663      	mov	r3, ip
    256c:	63dd      	str	r5, [r3, #60]	@ 0x3c
    256e:	f3bf 8f4f 	dsb	sy
    2572:	f3bf 8f6f 	isb	sy
    2576:	b672      	cpsid	i
    2578:	7eda      	ldrb	r2, [r3, #27]
    257a:	2a01      	cmp	r2, #1
    257c:	d032      	beq.n	0x25e4
    257e:	b662      	cpsie	i
    2580:	ab08      	add	r3, sp, #32
    2582:	1dd8      	adds	r0, r3, #7
    2584:	2303      	movs	r3, #3
    2586:	7003      	strb	r3, [r0, #0]
    2588:	ab08      	add	r3, sp, #32
    258a:	1d99      	adds	r1, r3, #6
    258c:	232e      	movs	r3, #46	@ 0x2e
    258e:	aa08      	add	r2, sp, #32
    2590:	700b      	strb	r3, [r1, #0]
    2592:	3205      	adds	r2, #5
    2594:	1d3b      	adds	r3, r7, #4
    2596:	f7ff fbdf 	bl	0x1d58
    259a:	e613      	b.n	0x21c4
    259c:	f7fd fe8c 	bl	0x2b8
    25a0:	2222      	movs	r2, #34	@ 0x22
    25a2:	9901      	ldr	r1, [sp, #4]
    25a4:	8c0b      	ldrh	r3, [r1, #32]
    25a6:	0a1b      	lsrs	r3, r3, #8
    25a8:	54bb      	strb	r3, [r7, r2]
    25aa:	2323      	movs	r3, #35	@ 0x23
    25ac:	8c0a      	ldrh	r2, [r1, #32]
    25ae:	54fa      	strb	r2, [r7, r3]
    25b0:	4b23      	ldr	r3, [pc, #140]	@ (0x2640)
    25b2:	2200      	movs	r2, #0
    25b4:	627b      	str	r3, [r7, #36]	@ 0x24
    25b6:	4b23      	ldr	r3, [pc, #140]	@ (0x2644)
    25b8:	853b      	strh	r3, [r7, #40]	@ 0x28
    25ba:	232a      	movs	r3, #42	@ 0x2a
    25bc:	54fa      	strb	r2, [r7, r3]
    25be:	ab08      	add	r3, sp, #32
    25c0:	1dd8      	adds	r0, r3, #7
    25c2:	1d99      	adds	r1, r3, #6
    25c4:	23a8      	movs	r3, #168	@ 0xa8
    25c6:	700b      	strb	r3, [r1, #0]
    25c8:	ab08      	add	r3, sp, #32
    25ca:	1d5a      	adds	r2, r3, #5
    25cc:	230a      	movs	r3, #10
    25ce:	7005      	strb	r5, [r0, #0]
    25d0:	7013      	strb	r3, [r2, #0]
    25d2:	1d3b      	adds	r3, r7, #4
    25d4:	f7ff fbc0 	bl	0x1d58
    25d8:	e5f4      	b.n	0x21c4
    25da:	683d      	ldr	r5, [r7, #0]
    25dc:	e6ca      	b.n	0x2374
    25de:	2301      	movs	r3, #1
    25e0:	9305      	str	r3, [sp, #20]
    25e2:	e757      	b.n	0x2494
    25e4:	4661      	mov	r1, ip
    25e6:	2306      	movs	r3, #6
    25e8:	76cb      	strb	r3, [r1, #27]
    25ea:	68cb      	ldr	r3, [r1, #12]
    25ec:	6818      	ldr	r0, [r3, #0]
    25ee:	2302      	movs	r3, #2
    25f0:	6043      	str	r3, [r0, #4]
    25f2:	0003      	movs	r3, r0
    25f4:	6042      	str	r2, [r0, #4]
    25f6:	7e89      	ldrb	r1, [r1, #26]
    25f8:	3318      	adds	r3, #24
    25fa:	008d      	lsls	r5, r1, #2
    25fc:	195b      	adds	r3, r3, r5
    25fe:	601a      	str	r2, [r3, #0]
    2600:	3202      	adds	r2, #2
    2602:	f7fe f861 	bl	0x6c8
    2606:	e7ba      	b.n	0x257e
    2608:	2300      	movs	r3, #0
    260a:	9305      	str	r3, [sp, #20]
    260c:	33ff      	adds	r3, #255	@ 0xff
    260e:	9304      	str	r3, [sp, #16]
    2610:	e740      	b.n	0x2494
    2612:	2302      	movs	r3, #2
    2614:	9305      	str	r3, [sp, #20]
    2616:	e73d      	b.n	0x2494
    2618:	23ff      	movs	r3, #255	@ 0xff
    261a:	2200      	movs	r2, #0
    261c:	e763      	b.n	0x24e6
    261e:	23ff      	movs	r3, #255	@ 0xff
    2620:	e7a2      	b.n	0x2568
    2622:	2207      	movs	r2, #7
    2624:	2107      	movs	r1, #7
    2626:	e6f0      	b.n	0x240a
    2628:	2300      	movs	r3, #0
    262a:	9803      	ldr	r0, [sp, #12]
    262c:	0019      	movs	r1, r3
    262e:	9e01      	ldr	r6, [sp, #4]
    2630:	9d02      	ldr	r5, [sp, #8]
    2632:	e520      	b.n	0x2076
    2634:	2300      	movs	r3, #0
    2636:	e528      	b.n	0x208a
    2638:	8000      	strh	r0, [r0, #0]
    263a:	4000      	ands	r0, r0
    263c:	e100      	b.n	0x2840
    263e:	e000      	b.n	0x2642
    2640:	0100      	lsls	r0, r0, #4
    2642:	0100      	lsls	r0, r0, #4
    2644:	5ae4      	ldrh	r4, [r4, r3]
    2646:	0000      	movs	r0, r0
    2648:	b580      	push	{r7, lr}
    264a:	af00      	add	r7, sp, #0
    264c:	4b01      	ldr	r3, [pc, #4]	@ (0x2654)
    264e:	0018      	movs	r0, r3
    2650:	46bd      	mov	sp, r7
    2652:	bd80      	pop	{r7, pc}
    2654:	0000      	movs	r0, r0
    2656:	1000      	asrs	r0, r0, #32
    2658:	b5f0      	push	{r4, r5, r6, r7, lr}
    265a:	201c      	movs	r0, #28
    265c:	b087      	sub	sp, #28
    265e:	f000 fb21 	bl	0x2ca4
    2662:	4ccd      	ldr	r4, [pc, #820]	@ (0x2998)
    2664:	2781      	movs	r7, #129	@ 0x81
    2666:	2504      	movs	r5, #4
    2668:	0023      	movs	r3, r4
    266a:	2600      	movs	r6, #0
    266c:	3308      	adds	r3, #8
    266e:	35ff      	adds	r5, #255	@ 0xff
    2670:	007f      	lsls	r7, r7, #1
    2672:	6003      	str	r3, [r0, #0]
    2674:	9001      	str	r0, [sp, #4]
    2676:	6046      	str	r6, [r0, #4]
    2678:	6087      	str	r7, [r0, #8]
    267a:	60c5      	str	r5, [r0, #12]
    267c:	0028      	movs	r0, r5
    267e:	f000 fb11 	bl	0x2ca4
    2682:	2103      	movs	r1, #3
    2684:	000b      	movs	r3, r1
    2686:	4242      	negs	r2, r0
    2688:	4013      	ands	r3, r2
    268a:	4211      	tst	r1, r2
    268c:	d100      	bne.n	0x2690
    268e:	e17d      	b.n	0x298c
    2690:	7006      	strb	r6, [r0, #0]
    2692:	0792      	lsls	r2, r2, #30
    2694:	d400      	bmi.n	0x2698
    2696:	e16f      	b.n	0x2978
    2698:	7046      	strb	r6, [r0, #1]
    269a:	2b03      	cmp	r3, #3
    269c:	d000      	beq.n	0x26a0
    269e:	e1ac      	b.n	0x29fa
    26a0:	2580      	movs	r5, #128	@ 0x80
    26a2:	7086      	strb	r6, [r0, #2]
    26a4:	1cc1      	adds	r1, r0, #3
    26a6:	006d      	lsls	r5, r5, #1
    26a8:	2604      	movs	r6, #4
    26aa:	2200      	movs	r2, #0
    26ac:	36ff      	adds	r6, #255	@ 0xff
    26ae:	1af6      	subs	r6, r6, r3
    26b0:	18c3      	adds	r3, r0, r3
    26b2:	1c5f      	adds	r7, r3, #1
    26b4:	37ff      	adds	r7, #255	@ 0xff
    26b6:	c304      	stmia	r3!, {r2}
    26b8:	429f      	cmp	r7, r3
    26ba:	d1fc      	bne.n	0x26b6
    26bc:	2380      	movs	r3, #128	@ 0x80
    26be:	3d01      	subs	r5, #1
    26c0:	3dff      	subs	r5, #255	@ 0xff
    26c2:	005b      	lsls	r3, r3, #1
    26c4:	429e      	cmp	r6, r3
    26c6:	d009      	beq.n	0x26dc
    26c8:	54ca      	strb	r2, [r1, r3]
    26ca:	2d01      	cmp	r5, #1
    26cc:	d006      	beq.n	0x26dc
    26ce:	2302      	movs	r3, #2
    26d0:	33ff      	adds	r3, #255	@ 0xff
    26d2:	54ca      	strb	r2, [r1, r3]
    26d4:	2d02      	cmp	r5, #2
    26d6:	d001      	beq.n	0x26dc
    26d8:	3301      	adds	r3, #1
    26da:	54ca      	strb	r2, [r1, r3]
    26dc:	9a01      	ldr	r2, [sp, #4]
    26de:	2500      	movs	r5, #0
    26e0:	6893      	ldr	r3, [r2, #8]
    26e2:	2603      	movs	r6, #3
    26e4:	3b03      	subs	r3, #3
    26e6:	6093      	str	r3, [r2, #8]
    26e8:	68d3      	ldr	r3, [r2, #12]
    26ea:	4fac      	ldr	r7, [pc, #688]	@ (0x299c)
    26ec:	3b03      	subs	r3, #3
    26ee:	60d3      	str	r3, [r2, #12]
    26f0:	0023      	movs	r3, r4
    26f2:	3334      	adds	r3, #52	@ 0x34
    26f4:	6013      	str	r3, [r2, #0]
    26f6:	4baa      	ldr	r3, [pc, #680]	@ (0x29a0)
    26f8:	6110      	str	r0, [r2, #16]
    26fa:	8295      	strh	r5, [r2, #20]
    26fc:	6196      	str	r6, [r2, #24]
    26fe:	2017      	movs	r0, #23
    2700:	3460      	adds	r4, #96	@ 0x60
    2702:	617b      	str	r3, [r7, #20]
    2704:	60fc      	str	r4, [r7, #12]
    2706:	613d      	str	r5, [r7, #16]
    2708:	61bd      	str	r5, [r7, #24]
    270a:	61fd      	str	r5, [r7, #28]
    270c:	f000 faca 	bl	0x2ca4
    2710:	0033      	movs	r3, r6
    2712:	4242      	negs	r2, r0
    2714:	4013      	ands	r3, r2
    2716:	4216      	tst	r6, r2
    2718:	d100      	bne.n	0x271c
    271a:	e130      	b.n	0x297e
    271c:	2100      	movs	r1, #0
    271e:	7001      	strb	r1, [r0, #0]
    2720:	0792      	lsls	r2, r2, #30
    2722:	d400      	bmi.n	0x2726
    2724:	e163      	b.n	0x29ee
    2726:	2200      	movs	r2, #0
    2728:	7042      	strb	r2, [r0, #1]
    272a:	2b03      	cmp	r3, #3
    272c:	d000      	beq.n	0x2730
    272e:	e181      	b.n	0x2a34
    2730:	2200      	movs	r2, #0
    2732:	2514      	movs	r5, #20
    2734:	7082      	strb	r2, [r0, #2]
    2736:	1cc4      	adds	r4, r0, #3
    2738:	2117      	movs	r1, #23
    273a:	1ac9      	subs	r1, r1, r3
    273c:	18c3      	adds	r3, r0, r3
    273e:	001a      	movs	r2, r3
    2740:	2600      	movs	r6, #0
    2742:	c240      	stmia	r2!, {r6}
    2744:	2600      	movs	r6, #0
    2746:	3d14      	subs	r5, #20
    2748:	6016      	str	r6, [r2, #0]
    274a:	609e      	str	r6, [r3, #8]
    274c:	60de      	str	r6, [r3, #12]
    274e:	611e      	str	r6, [r3, #16]
    2750:	2914      	cmp	r1, #20
    2752:	d006      	beq.n	0x2762
    2754:	7526      	strb	r6, [r4, #20]
    2756:	2d01      	cmp	r5, #1
    2758:	d003      	beq.n	0x2762
    275a:	7566      	strb	r6, [r4, #21]
    275c:	2d02      	cmp	r5, #2
    275e:	d000      	beq.n	0x2762
    2760:	75a6      	strb	r6, [r4, #22]
    2762:	6278      	str	r0, [r7, #36]	@ 0x24
    2764:	2017      	movs	r0, #23
    2766:	f000 fa9d 	bl	0x2ca4
    276a:	2203      	movs	r2, #3
    276c:	0013      	movs	r3, r2
    276e:	4241      	negs	r1, r0
    2770:	400b      	ands	r3, r1
    2772:	420a      	tst	r2, r1
    2774:	d100      	bne.n	0x2778
    2776:	e133      	b.n	0x29e0
    2778:	2200      	movs	r2, #0
    277a:	7002      	strb	r2, [r0, #0]
    277c:	0789      	lsls	r1, r1, #30
    277e:	d400      	bmi.n	0x2782
    2780:	e12b      	b.n	0x29da
    2782:	7042      	strb	r2, [r0, #1]
    2784:	2b03      	cmp	r3, #3
    2786:	d000      	beq.n	0x278a
    2788:	e14c      	b.n	0x2a24
    278a:	2114      	movs	r1, #20
    278c:	7082      	strb	r2, [r0, #2]
    278e:	1cc6      	adds	r6, r0, #3
    2790:	2517      	movs	r5, #23
    2792:	1aed      	subs	r5, r5, r3
    2794:	18c3      	adds	r3, r0, r3
    2796:	001c      	movs	r4, r3
    2798:	2200      	movs	r2, #0
    279a:	c404      	stmia	r4!, {r2}
    279c:	2200      	movs	r2, #0
    279e:	3914      	subs	r1, #20
    27a0:	6022      	str	r2, [r4, #0]
    27a2:	609a      	str	r2, [r3, #8]
    27a4:	60da      	str	r2, [r3, #12]
    27a6:	611a      	str	r2, [r3, #16]
    27a8:	2d14      	cmp	r5, #20
    27aa:	d006      	beq.n	0x27ba
    27ac:	7532      	strb	r2, [r6, #20]
    27ae:	2901      	cmp	r1, #1
    27b0:	d003      	beq.n	0x27ba
    27b2:	7572      	strb	r2, [r6, #21]
    27b4:	2902      	cmp	r1, #2
    27b6:	d000      	beq.n	0x27ba
    27b8:	75b2      	strb	r2, [r6, #22]
    27ba:	62b8      	str	r0, [r7, #40]	@ 0x28
    27bc:	2017      	movs	r0, #23
    27be:	f000 fa71 	bl	0x2ca4
    27c2:	2203      	movs	r2, #3
    27c4:	0013      	movs	r3, r2
    27c6:	4241      	negs	r1, r0
    27c8:	400b      	ands	r3, r1
    27ca:	420a      	tst	r2, r1
    27cc:	d100      	bne.n	0x27d0
    27ce:	e0fd      	b.n	0x29cc
    27d0:	2200      	movs	r2, #0
    27d2:	7002      	strb	r2, [r0, #0]
    27d4:	0789      	lsls	r1, r1, #30
    27d6:	d400      	bmi.n	0x27da
    27d8:	e0f5      	b.n	0x29c6
    27da:	7042      	strb	r2, [r0, #1]
    27dc:	2b03      	cmp	r3, #3
    27de:	d000      	beq.n	0x27e2
    27e0:	e118      	b.n	0x2a14
    27e2:	2114      	movs	r1, #20
    27e4:	7082      	strb	r2, [r0, #2]
    27e6:	1cc6      	adds	r6, r0, #3
    27e8:	2517      	movs	r5, #23
    27ea:	1aed      	subs	r5, r5, r3
    27ec:	18c3      	adds	r3, r0, r3
    27ee:	001c      	movs	r4, r3
    27f0:	2200      	movs	r2, #0
    27f2:	c404      	stmia	r4!, {r2}
    27f4:	2200      	movs	r2, #0
    27f6:	3914      	subs	r1, #20
    27f8:	6022      	str	r2, [r4, #0]
    27fa:	609a      	str	r2, [r3, #8]
    27fc:	60da      	str	r2, [r3, #12]
    27fe:	611a      	str	r2, [r3, #16]
    2800:	2d14      	cmp	r5, #20
    2802:	d006      	beq.n	0x2812
    2804:	7532      	strb	r2, [r6, #20]
    2806:	2901      	cmp	r1, #1
    2808:	d003      	beq.n	0x2812
    280a:	7572      	strb	r2, [r6, #21]
    280c:	2902      	cmp	r1, #2
    280e:	d000      	beq.n	0x2812
    2810:	75b2      	strb	r2, [r6, #22]
    2812:	4b64      	ldr	r3, [pc, #400]	@ (0x29a4)
    2814:	62f8      	str	r0, [r7, #44]	@ 0x2c
    2816:	9304      	str	r3, [sp, #16]
    2818:	2060      	movs	r0, #96	@ 0x60
    281a:	3314      	adds	r3, #20
    281c:	60fb      	str	r3, [r7, #12]
    281e:	f000 fa41 	bl	0x2ca4
    2822:	003b      	movs	r3, r7
    2824:	330c      	adds	r3, #12
    2826:	6083      	str	r3, [r0, #8]
    2828:	003b      	movs	r3, r7
    282a:	2600      	movs	r6, #0
    282c:	335c      	adds	r3, #92	@ 0x5c
    282e:	60c3      	str	r3, [r0, #12]
    2830:	4b5d      	ldr	r3, [pc, #372]	@ (0x29a8)
    2832:	8306      	strh	r6, [r0, #24]
    2834:	6103      	str	r3, [r0, #16]
    2836:	4b5d      	ldr	r3, [pc, #372]	@ (0x29ac)
    2838:	0004      	movs	r4, r0
    283a:	6143      	str	r3, [r0, #20]
    283c:	2017      	movs	r0, #23
    283e:	f000 fa31 	bl	0x2ca4
    2842:	2103      	movs	r1, #3
    2844:	000b      	movs	r3, r1
    2846:	4242      	negs	r2, r0
    2848:	2500      	movs	r5, #0
    284a:	4013      	ands	r3, r2
    284c:	4211      	tst	r1, r2
    284e:	d100      	bne.n	0x2852
    2850:	e0b2      	b.n	0x29b8
    2852:	7005      	strb	r5, [r0, #0]
    2854:	0792      	lsls	r2, r2, #30
    2856:	d400      	bmi.n	0x285a
    2858:	e09a      	b.n	0x2990
    285a:	7045      	strb	r5, [r0, #1]
    285c:	2b03      	cmp	r3, #3
    285e:	d000      	beq.n	0x2862
    2860:	e0cf      	b.n	0x2a02
    2862:	2214      	movs	r2, #20
    2864:	4694      	mov	ip, r2
    2866:	7085      	strb	r5, [r0, #2]
    2868:	1cc5      	adds	r5, r0, #3
    286a:	2117      	movs	r1, #23
    286c:	1ac9      	subs	r1, r1, r3
    286e:	18c3      	adds	r3, r0, r3
    2870:	001a      	movs	r2, r3
    2872:	2600      	movs	r6, #0
    2874:	c240      	stmia	r2!, {r6}
    2876:	2600      	movs	r6, #0
    2878:	6016      	str	r6, [r2, #0]
    287a:	609e      	str	r6, [r3, #8]
    287c:	60de      	str	r6, [r3, #12]
    287e:	611e      	str	r6, [r3, #16]
    2880:	4663      	mov	r3, ip
    2882:	3b14      	subs	r3, #20
    2884:	2914      	cmp	r1, #20
    2886:	d006      	beq.n	0x2896
    2888:	752e      	strb	r6, [r5, #20]
    288a:	2b01      	cmp	r3, #1
    288c:	d003      	beq.n	0x2896
    288e:	756e      	strb	r6, [r5, #21]
    2890:	2b02      	cmp	r3, #2
    2892:	d000      	beq.n	0x2896
    2894:	75ae      	strb	r6, [r5, #22]
    2896:	2203      	movs	r2, #3
    2898:	7e63      	ldrb	r3, [r4, #25]
    289a:	6222      	str	r2, [r4, #32]
    289c:	3302      	adds	r3, #2
    289e:	4013      	ands	r3, r2
    28a0:	76a3      	strb	r3, [r4, #26]
    28a2:	2300      	movs	r3, #0
    28a4:	76e3      	strb	r3, [r4, #27]
    28a6:	4b42      	ldr	r3, [pc, #264]	@ (0x29b0)
    28a8:	62a2      	str	r2, [r4, #40]	@ 0x28
    28aa:	001a      	movs	r2, r3
    28ac:	6420      	str	r0, [r4, #64]	@ 0x40
    28ae:	9305      	str	r3, [sp, #20]
    28b0:	68a0      	ldr	r0, [r4, #8]
    28b2:	6963      	ldr	r3, [r4, #20]
    28b4:	6053      	str	r3, [r2, #4]
    28b6:	6803      	ldr	r3, [r0, #0]
    28b8:	6b5b      	ldr	r3, [r3, #52]	@ 0x34
    28ba:	4798      	blx	r3
    28bc:	0005      	movs	r5, r0
    28be:	f000 f9f1 	bl	0x2ca4
    28c2:	1e6a      	subs	r2, r5, #1
    28c4:	1e11      	subs	r1, r2, #0
    28c6:	db42      	blt.n	0x294e
    28c8:	1eab      	subs	r3, r5, #2
    28ca:	469c      	mov	ip, r3
    28cc:	2301      	movs	r3, #1
    28ce:	9302      	str	r3, [sp, #8]
    28d0:	4663      	mov	r3, ip
    28d2:	3301      	adds	r3, #1
    28d4:	db00      	blt.n	0x28d8
    28d6:	9502      	str	r5, [sp, #8]
    28d8:	2303      	movs	r3, #3
    28da:	4246      	negs	r6, r0
    28dc:	4033      	ands	r3, r6
    28de:	2a05      	cmp	r2, #5
    28e0:	d800      	bhi.n	0x28e4
    28e2:	e0b4      	b.n	0x2a4e
    28e4:	9003      	str	r0, [sp, #12]
    28e6:	2b00      	cmp	r3, #0
    28e8:	d00e      	beq.n	0x2908
    28ea:	2200      	movs	r2, #0
    28ec:	7002      	strb	r2, [r0, #0]
    28ee:	07b6      	lsls	r6, r6, #30
    28f0:	d400      	bmi.n	0x28f4
    28f2:	e07f      	b.n	0x29f4
    28f4:	7042      	strb	r2, [r0, #1]
    28f6:	2b03      	cmp	r3, #3
    28f8:	d000      	beq.n	0x28fc
    28fa:	e0a3      	b.n	0x2a44
    28fc:	7082      	strb	r2, [r0, #2]
    28fe:	1f2a      	subs	r2, r5, #4
    2900:	4694      	mov	ip, r2
    2902:	1cc2      	adds	r2, r0, #3
    2904:	9203      	str	r2, [sp, #12]
    2906:	4661      	mov	r1, ip
    2908:	2600      	movs	r6, #0
    290a:	9a02      	ldr	r2, [sp, #8]
    290c:	1ad5      	subs	r5, r2, r3
    290e:	08aa      	lsrs	r2, r5, #2
    2910:	18c3      	adds	r3, r0, r3
    2912:	0092      	lsls	r2, r2, #2
    2914:	18d2      	adds	r2, r2, r3
    2916:	c340      	stmia	r3!, {r6}
    2918:	429a      	cmp	r2, r3
    291a:	d1fc      	bne.n	0x2916
    291c:	2203      	movs	r2, #3
    291e:	002e      	movs	r6, r5
    2920:	9b03      	ldr	r3, [sp, #12]
    2922:	4396      	bics	r6, r2
    2924:	199b      	adds	r3, r3, r6
    2926:	1b89      	subs	r1, r1, r6
    2928:	422a      	tst	r2, r5
    292a:	d010      	beq.n	0x294e
    292c:	2200      	movs	r2, #0
    292e:	701a      	strb	r2, [r3, #0]
    2930:	2900      	cmp	r1, #0
    2932:	d00c      	beq.n	0x294e
    2934:	705a      	strb	r2, [r3, #1]
    2936:	2901      	cmp	r1, #1
    2938:	d009      	beq.n	0x294e
    293a:	709a      	strb	r2, [r3, #2]
    293c:	2902      	cmp	r1, #2
    293e:	d006      	beq.n	0x294e
    2940:	70da      	strb	r2, [r3, #3]
    2942:	2903      	cmp	r1, #3
    2944:	d003      	beq.n	0x294e
    2946:	711a      	strb	r2, [r3, #4]
    2948:	2904      	cmp	r1, #4
    294a:	d000      	beq.n	0x294e
    294c:	715a      	strb	r2, [r3, #5]
    294e:	9b01      	ldr	r3, [sp, #4]
    2950:	2154      	movs	r1, #84	@ 0x54
    2952:	63fb      	str	r3, [r7, #60]	@ 0x3c
    2954:	2300      	movs	r3, #0
    2956:	4a17      	ldr	r2, [pc, #92]	@ (0x29b4)
    2958:	643b      	str	r3, [r7, #64]	@ 0x40
    295a:	647b      	str	r3, [r7, #68]	@ 0x44
    295c:	64bb      	str	r3, [r7, #72]	@ 0x48
    295e:	527b      	strh	r3, [r7, r1]
    2960:	65bb      	str	r3, [r7, #88]	@ 0x58
    2962:	9b05      	ldr	r3, [sp, #20]
    2964:	6020      	str	r0, [r4, #0]
    2966:	601a      	str	r2, [r3, #0]
    2968:	9b04      	ldr	r3, [sp, #16]
    296a:	637c      	str	r4, [r7, #52]	@ 0x34
    296c:	3354      	adds	r3, #84	@ 0x54
    296e:	63ba      	str	r2, [r7, #56]	@ 0x38
    2970:	60bc      	str	r4, [r7, #8]
    2972:	60fb      	str	r3, [r7, #12]
    2974:	b007      	add	sp, #28
    2976:	bdf0      	pop	{r4, r5, r6, r7, pc}
    2978:	003d      	movs	r5, r7
    297a:	1c41      	adds	r1, r0, #1
    297c:	e694      	b.n	0x26a8
    297e:	0002      	movs	r2, r0
    2980:	2117      	movs	r1, #23
    2982:	c208      	stmia	r2!, {r3}
    2984:	0004      	movs	r4, r0
    2986:	0003      	movs	r3, r0
    2988:	000d      	movs	r5, r1
    298a:	e6db      	b.n	0x2744
    298c:	0001      	movs	r1, r0
    298e:	e68b      	b.n	0x26a8
    2990:	2216      	movs	r2, #22
    2992:	1c45      	adds	r5, r0, #1
    2994:	4694      	mov	ip, r2
    2996:	e768      	b.n	0x286a
    2998:	3078      	adds	r0, #120	@ 0x78
    299a:	0000      	movs	r0, r0
    299c:	0070      	lsls	r0, r6, #1
    299e:	1000      	asrs	r0, r0, #32
    29a0:	ffff 0000 	vaddl.u<illegal width 64>	q8, d15, d0
    29a4:	30f8      	adds	r0, #248	@ 0xf8
    29a6:	0000      	movs	r0, r0
    29a8:	c228      	stmia	r2!, {r3, r5}
    29aa:	0001      	movs	r1, r0
    29ac:	8229      	strh	r1, [r5, #16]
    29ae:	0069      	lsls	r1, r5, #1
    29b0:	0014      	movs	r4, r2
    29b2:	1000      	asrs	r0, r0, #32
    29b4:	8240      	strh	r0, [r0, #18]
    29b6:	005c      	lsls	r4, r3, #1
    29b8:	0002      	movs	r2, r0
    29ba:	3114      	adds	r1, #20
    29bc:	c208      	stmia	r2!, {r3}
    29be:	0005      	movs	r5, r0
    29c0:	0003      	movs	r3, r0
    29c2:	468c      	mov	ip, r1
    29c4:	e757      	b.n	0x2876
    29c6:	2116      	movs	r1, #22
    29c8:	1c46      	adds	r6, r0, #1
    29ca:	e70d      	b.n	0x27e8
    29cc:	0004      	movs	r4, r0
    29ce:	2517      	movs	r5, #23
    29d0:	c408      	stmia	r4!, {r3}
    29d2:	0006      	movs	r6, r0
    29d4:	0003      	movs	r3, r0
    29d6:	0029      	movs	r1, r5
    29d8:	e70c      	b.n	0x27f4
    29da:	2116      	movs	r1, #22
    29dc:	1c46      	adds	r6, r0, #1
    29de:	e6d7      	b.n	0x2790
    29e0:	0004      	movs	r4, r0
    29e2:	2517      	movs	r5, #23
    29e4:	c408      	stmia	r4!, {r3}
    29e6:	0006      	movs	r6, r0
    29e8:	0003      	movs	r3, r0
    29ea:	0029      	movs	r1, r5
    29ec:	e6d6      	b.n	0x279c
    29ee:	2516      	movs	r5, #22
    29f0:	1c44      	adds	r4, r0, #1
    29f2:	e6a1      	b.n	0x2738
    29f4:	1c42      	adds	r2, r0, #1
    29f6:	9203      	str	r2, [sp, #12]
    29f8:	e785      	b.n	0x2906
    29fa:	2502      	movs	r5, #2
    29fc:	1c81      	adds	r1, r0, #2
    29fe:	35ff      	adds	r5, #255	@ 0xff
    2a00:	e652      	b.n	0x26a8
    2a02:	2117      	movs	r1, #23
    2a04:	1ac9      	subs	r1, r1, r3
    2a06:	18c3      	adds	r3, r0, r3
    2a08:	001a      	movs	r2, r3
    2a0a:	c240      	stmia	r2!, {r6}
    2a0c:	2615      	movs	r6, #21
    2a0e:	1c85      	adds	r5, r0, #2
    2a10:	46b4      	mov	ip, r6
    2a12:	e730      	b.n	0x2876
    2a14:	2517      	movs	r5, #23
    2a16:	1aed      	subs	r5, r5, r3
    2a18:	18c3      	adds	r3, r0, r3
    2a1a:	001c      	movs	r4, r3
    2a1c:	2115      	movs	r1, #21
    2a1e:	1c86      	adds	r6, r0, #2
    2a20:	c404      	stmia	r4!, {r2}
    2a22:	e6e7      	b.n	0x27f4
    2a24:	2517      	movs	r5, #23
    2a26:	1aed      	subs	r5, r5, r3
    2a28:	18c3      	adds	r3, r0, r3
    2a2a:	001c      	movs	r4, r3
    2a2c:	2115      	movs	r1, #21
    2a2e:	1c86      	adds	r6, r0, #2
    2a30:	c404      	stmia	r4!, {r2}
    2a32:	e6b3      	b.n	0x279c
    2a34:	2117      	movs	r1, #23
    2a36:	1ac9      	subs	r1, r1, r3
    2a38:	18c3      	adds	r3, r0, r3
    2a3a:	001a      	movs	r2, r3
    2a3c:	1c84      	adds	r4, r0, #2
    2a3e:	c220      	stmia	r2!, {r5}
    2a40:	2515      	movs	r5, #21
    2a42:	e67f      	b.n	0x2744
    2a44:	1eea      	subs	r2, r5, #3
    2a46:	4694      	mov	ip, r2
    2a48:	1c82      	adds	r2, r0, #2
    2a4a:	9203      	str	r2, [sp, #12]
    2a4c:	e75b      	b.n	0x2906
    2a4e:	0003      	movs	r3, r0
    2a50:	e76c      	b.n	0x292c
    2a52:	b510      	push	{r4, lr}
    2a54:	2400      	movs	r4, #0
    2a56:	4a18      	ldr	r2, [pc, #96]	@ (0x2ab8)
    2a58:	4b18      	ldr	r3, [pc, #96]	@ (0x2abc)
    2a5a:	7114      	strb	r4, [r2, #4]
    2a5c:	4a18      	ldr	r2, [pc, #96]	@ (0x2ac0)
    2a5e:	2101      	movs	r1, #1
    2a60:	665a      	str	r2, [r3, #100]	@ 0x64
    2a62:	001a      	movs	r2, r3
    2a64:	325c      	adds	r2, #92	@ 0x5c
    2a66:	7111      	strb	r1, [r2, #4]
    2a68:	4a16      	ldr	r2, [pc, #88]	@ (0x2ac4)
    2a6a:	1849      	adds	r1, r1, r1
    2a6c:	65da      	str	r2, [r3, #92]	@ 0x5c
    2a6e:	001a      	movs	r2, r3
    2a70:	326c      	adds	r2, #108	@ 0x6c
    2a72:	7111      	strb	r1, [r2, #4]
    2a74:	4a14      	ldr	r2, [pc, #80]	@ (0x2ac8)
    2a76:	3101      	adds	r1, #1
    2a78:	66da      	str	r2, [r3, #108]	@ 0x6c
    2a7a:	001a      	movs	r2, r3
    2a7c:	3274      	adds	r2, #116	@ 0x74
    2a7e:	7111      	strb	r1, [r2, #4]
    2a80:	4a12      	ldr	r2, [pc, #72]	@ (0x2acc)
    2a82:	4913      	ldr	r1, [pc, #76]	@ (0x2ad0)
    2a84:	675a      	str	r2, [r3, #116]	@ 0x74
    2a86:	22fa      	movs	r2, #250	@ 0xfa
    2a88:	4b12      	ldr	r3, [pc, #72]	@ (0x2ad4)
    2a8a:	0092      	lsls	r2, r2, #2
    2a8c:	605a      	str	r2, [r3, #4]
    2a8e:	4a12      	ldr	r2, [pc, #72]	@ (0x2ad8)
    2a90:	4812      	ldr	r0, [pc, #72]	@ (0x2adc)
    2a92:	601a      	str	r2, [r3, #0]
    2a94:	33ff      	adds	r3, #255	@ 0xff
    2a96:	765c      	strb	r4, [r3, #25]
    2a98:	f7fd fefc 	bl	0x894
    2a9c:	4910      	ldr	r1, [pc, #64]	@ (0x2ae0)
    2a9e:	4811      	ldr	r0, [pc, #68]	@ (0x2ae4)
    2aa0:	f7fd fef8 	bl	0x894
    2aa4:	f7ff fdd0 	bl	0x2648
    2aa8:	4b0f      	ldr	r3, [pc, #60]	@ (0x2ae8)
    2aaa:	6218      	str	r0, [r3, #32]
    2aac:	f7ff fdd4 	bl	0x2658
    2ab0:	4b0e      	ldr	r3, [pc, #56]	@ (0x2aec)
    2ab2:	60dc      	str	r4, [r3, #12]
    2ab4:	67dc      	str	r4, [r3, #124]	@ 0x7c
    2ab6:	bd10      	pop	{r4, pc}
    2ab8:	00d4      	lsls	r4, r2, #3
    2aba:	1000      	asrs	r0, r0, #32
    2abc:	0070      	lsls	r0, r6, #1
    2abe:	1000      	asrs	r0, r0, #32
    2ac0:	c000      	stmia	r0!, {}
    2ac2:	4000      	ands	r0, r0
    2ac4:	0000      	movs	r0, r0
    2ac6:	4001      	ands	r1, r0
    2ac8:	4000      	ands	r0, r0
    2aca:	4001      	ands	r1, r0
    2acc:	8000      	strh	r0, [r0, #0]
    2ace:	4001      	ands	r1, r0
    2ad0:	1000      	asrs	r0, r0, #32
    2ad2:	003c      	movs	r4, r7
    2ad4:	0194      	lsls	r4, r2, #6
    2ad6:	1000      	asrs	r0, r0, #32
    2ad8:	318c      	adds	r1, #140	@ 0x8c
    2ada:	0000      	movs	r0, r0
    2adc:	8247      	strh	r7, [r0, #18]
    2ade:	0079      	lsls	r1, r7, #1
    2ae0:	6000      	str	r0, [r0, #0]
    2ae2:	0040      	lsls	r0, r0, #1
    2ae4:	8248      	strh	r0, [r1, #18]
    2ae6:	0081      	lsls	r1, r0, #2
    2ae8:	0170      	lsls	r0, r6, #5
    2aea:	1000      	asrs	r0, r0, #32
    2aec:	00f0      	lsls	r0, r6, #3
    2aee:	1000      	asrs	r0, r0, #32
    2af0:	218e      	movs	r1, #142	@ 0x8e
    2af2:	2020      	movs	r0, #32
    2af4:	4b1a      	ldr	r3, [pc, #104]	@ (0x2b60)
    2af6:	0089      	lsls	r1, r1, #2
    2af8:	585a      	ldr	r2, [r3, r1]
    2afa:	4382      	bics	r2, r0
    2afc:	505a      	str	r2, [r3, r1]
    2afe:	2200      	movs	r2, #0
    2b00:	3971      	subs	r1, #113	@ 0x71
    2b02:	621a      	str	r2, [r3, #32]
    2b04:	39ff      	subs	r1, #255	@ 0xff
    2b06:	46c0      	nop			@ (mov r8, r8)
    2b08:	3901      	subs	r1, #1
    2b0a:	2900      	cmp	r1, #0
    2b0c:	d1fb      	bne.n	0x2b06
    2b0e:	2201      	movs	r2, #1
    2b10:	641a      	str	r2, [r3, #64]	@ 0x40
    2b12:	645a      	str	r2, [r3, #68]	@ 0x44
    2b14:	6459      	str	r1, [r3, #68]	@ 0x44
    2b16:	645a      	str	r2, [r3, #68]	@ 0x44
    2b18:	6c59      	ldr	r1, [r3, #68]	@ 0x44
    2b1a:	4211      	tst	r1, r2
    2b1c:	d0fc      	beq.n	0x2b18
    2b1e:	2223      	movs	r2, #35	@ 0x23
    2b20:	218e      	movs	r1, #142	@ 0x8e
    2b22:	2080      	movs	r0, #128	@ 0x80
    2b24:	609a      	str	r2, [r3, #8]
    2b26:	0089      	lsls	r1, r1, #2
    2b28:	585a      	ldr	r2, [r3, r1]
    2b2a:	4382      	bics	r2, r0
    2b2c:	505a      	str	r2, [r3, r1]
    2b2e:	2201      	movs	r2, #1
    2b30:	68d9      	ldr	r1, [r3, #12]
    2b32:	4211      	tst	r1, r2
    2b34:	d0fc      	beq.n	0x2b30
    2b36:	2103      	movs	r1, #3
    2b38:	6719      	str	r1, [r3, #112]	@ 0x70
    2b3a:	2100      	movs	r1, #0
    2b3c:	675a      	str	r2, [r3, #116]	@ 0x74
    2b3e:	6759      	str	r1, [r3, #116]	@ 0x74
    2b40:	675a      	str	r2, [r3, #116]	@ 0x74
    2b42:	2201      	movs	r2, #1
    2b44:	6f59      	ldr	r1, [r3, #116]	@ 0x74
    2b46:	4211      	tst	r1, r2
    2b48:	d0fc      	beq.n	0x2b44
    2b4a:	679a      	str	r2, [r3, #120]	@ 0x78
    2b4c:	4905      	ldr	r1, [pc, #20]	@ (0x2b64)
    2b4e:	4b06      	ldr	r3, [pc, #24]	@ (0x2b68)
    2b50:	67d9      	str	r1, [r3, #124]	@ 0x7c
    2b52:	4b06      	ldr	r3, [pc, #24]	@ (0x2b6c)
    2b54:	601a      	str	r2, [r3, #0]
    2b56:	4b06      	ldr	r3, [pc, #24]	@ (0x2b70)
    2b58:	601a      	str	r2, [r3, #0]
    2b5a:	4b06      	ldr	r3, [pc, #24]	@ (0x2b74)
    2b5c:	601a      	str	r2, [r3, #0]
    2b5e:	4770      	bx	lr
    2b60:	8000      	strh	r0, [r0, #0]
    2b62:	4004      	ands	r4, r0
    2b64:	005f      	lsls	r7, r3, #1
    2b66:	0001      	movs	r1, r0
    2b68:	8004      	strh	r4, [r0, #0]
    2b6a:	4004      	ands	r4, r0
    2b6c:	8094      	strh	r4, [r2, #4]
    2b6e:	4004      	ands	r4, r0
    2b70:	8098      	strh	r0, [r3, #4]
    2b72:	4004      	ands	r4, r0
    2b74:	809c      	strh	r4, [r3, #4]
    2b76:	4004      	ands	r4, r0
    2b78:	2200      	movs	r2, #0
    2b7a:	0843      	lsrs	r3, r0, #1
    2b7c:	428b      	cmp	r3, r1
    2b7e:	d374      	bcc.n	0x2c6a
    2b80:	0903      	lsrs	r3, r0, #4
    2b82:	428b      	cmp	r3, r1
    2b84:	d35f      	bcc.n	0x2c46
    2b86:	0a03      	lsrs	r3, r0, #8
    2b88:	428b      	cmp	r3, r1
    2b8a:	d344      	bcc.n	0x2c16
    2b8c:	0b03      	lsrs	r3, r0, #12
    2b8e:	428b      	cmp	r3, r1
    2b90:	d328      	bcc.n	0x2be4
    2b92:	0c03      	lsrs	r3, r0, #16
    2b94:	428b      	cmp	r3, r1
    2b96:	d30d      	bcc.n	0x2bb4
    2b98:	22ff      	movs	r2, #255	@ 0xff
    2b9a:	0209      	lsls	r1, r1, #8
    2b9c:	ba12      	rev	r2, r2
    2b9e:	0c03      	lsrs	r3, r0, #16
    2ba0:	428b      	cmp	r3, r1
    2ba2:	d302      	bcc.n	0x2baa
    2ba4:	1212      	asrs	r2, r2, #8
    2ba6:	0209      	lsls	r1, r1, #8
    2ba8:	d065      	beq.n	0x2c76
    2baa:	0b03      	lsrs	r3, r0, #12
    2bac:	428b      	cmp	r3, r1
    2bae:	d319      	bcc.n	0x2be4
    2bb0:	e000      	b.n	0x2bb4
    2bb2:	0a09      	lsrs	r1, r1, #8
    2bb4:	0bc3      	lsrs	r3, r0, #15
    2bb6:	428b      	cmp	r3, r1
    2bb8:	d301      	bcc.n	0x2bbe
    2bba:	03cb      	lsls	r3, r1, #15
    2bbc:	1ac0      	subs	r0, r0, r3
    2bbe:	4152      	adcs	r2, r2
    2bc0:	0b83      	lsrs	r3, r0, #14
    2bc2:	428b      	cmp	r3, r1
    2bc4:	d301      	bcc.n	0x2bca
    2bc6:	038b      	lsls	r3, r1, #14
    2bc8:	1ac0      	subs	r0, r0, r3
    2bca:	4152      	adcs	r2, r2
    2bcc:	0b43      	lsrs	r3, r0, #13
    2bce:	428b      	cmp	r3, r1
    2bd0:	d301      	bcc.n	0x2bd6
    2bd2:	034b      	lsls	r3, r1, #13
    2bd4:	1ac0      	subs	r0, r0, r3
    2bd6:	4152      	adcs	r2, r2
    2bd8:	0b03      	lsrs	r3, r0, #12
    2bda:	428b      	cmp	r3, r1
    2bdc:	d301      	bcc.n	0x2be2
    2bde:	030b      	lsls	r3, r1, #12
    2be0:	1ac0      	subs	r0, r0, r3
    2be2:	4152      	adcs	r2, r2
    2be4:	0ac3      	lsrs	r3, r0, #11
    2be6:	428b      	cmp	r3, r1
    2be8:	d301      	bcc.n	0x2bee
    2bea:	02cb      	lsls	r3, r1, #11
    2bec:	1ac0      	subs	r0, r0, r3
    2bee:	4152      	adcs	r2, r2
    2bf0:	0a83      	lsrs	r3, r0, #10
    2bf2:	428b      	cmp	r3, r1
    2bf4:	d301      	bcc.n	0x2bfa
    2bf6:	028b      	lsls	r3, r1, #10
    2bf8:	1ac0      	subs	r0, r0, r3
    2bfa:	4152      	adcs	r2, r2
    2bfc:	0a43      	lsrs	r3, r0, #9
    2bfe:	428b      	cmp	r3, r1
    2c00:	d301      	bcc.n	0x2c06
    2c02:	024b      	lsls	r3, r1, #9
    2c04:	1ac0      	subs	r0, r0, r3
    2c06:	4152      	adcs	r2, r2
    2c08:	0a03      	lsrs	r3, r0, #8
    2c0a:	428b      	cmp	r3, r1
    2c0c:	d301      	bcc.n	0x2c12
    2c0e:	020b      	lsls	r3, r1, #8
    2c10:	1ac0      	subs	r0, r0, r3
    2c12:	4152      	adcs	r2, r2
    2c14:	d2cd      	bcs.n	0x2bb2
    2c16:	09c3      	lsrs	r3, r0, #7
    2c18:	428b      	cmp	r3, r1
    2c1a:	d301      	bcc.n	0x2c20
    2c1c:	01cb      	lsls	r3, r1, #7
    2c1e:	1ac0      	subs	r0, r0, r3
    2c20:	4152      	adcs	r2, r2
    2c22:	0983      	lsrs	r3, r0, #6
    2c24:	428b      	cmp	r3, r1
    2c26:	d301      	bcc.n	0x2c2c
    2c28:	018b      	lsls	r3, r1, #6
    2c2a:	1ac0      	subs	r0, r0, r3
    2c2c:	4152      	adcs	r2, r2
    2c2e:	0943      	lsrs	r3, r0, #5
    2c30:	428b      	cmp	r3, r1
    2c32:	d301      	bcc.n	0x2c38
    2c34:	014b      	lsls	r3, r1, #5
    2c36:	1ac0      	subs	r0, r0, r3
    2c38:	4152      	adcs	r2, r2
    2c3a:	0903      	lsrs	r3, r0, #4
    2c3c:	428b      	cmp	r3, r1
    2c3e:	d301      	bcc.n	0x2c44
    2c40:	010b      	lsls	r3, r1, #4
    2c42:	1ac0      	subs	r0, r0, r3
    2c44:	4152      	adcs	r2, r2
    2c46:	08c3      	lsrs	r3, r0, #3
    2c48:	428b      	cmp	r3, r1
    2c4a:	d301      	bcc.n	0x2c50
    2c4c:	00cb      	lsls	r3, r1, #3
    2c4e:	1ac0      	subs	r0, r0, r3
    2c50:	4152      	adcs	r2, r2
    2c52:	0883      	lsrs	r3, r0, #2
    2c54:	428b      	cmp	r3, r1
    2c56:	d301      	bcc.n	0x2c5c
    2c58:	008b      	lsls	r3, r1, #2
    2c5a:	1ac0      	subs	r0, r0, r3
    2c5c:	4152      	adcs	r2, r2
    2c5e:	0843      	lsrs	r3, r0, #1
    2c60:	428b      	cmp	r3, r1
    2c62:	d301      	bcc.n	0x2c68
    2c64:	004b      	lsls	r3, r1, #1
    2c66:	1ac0      	subs	r0, r0, r3
    2c68:	4152      	adcs	r2, r2
    2c6a:	1a41      	subs	r1, r0, r1
    2c6c:	d200      	bcs.n	0x2c70
    2c6e:	4601      	mov	r1, r0
    2c70:	4152      	adcs	r2, r2
    2c72:	4610      	mov	r0, r2
    2c74:	4770      	bx	lr
    2c76:	e7ff      	b.n	0x2c78
    2c78:	b501      	push	{r0, lr}
    2c7a:	2000      	movs	r0, #0
    2c7c:	f000 f806 	bl	0x2c8c
    2c80:	bd02      	pop	{r1, pc}
    2c82:	46c0      	nop			@ (mov r8, r8)
    2c84:	2900      	cmp	r1, #0
    2c86:	d0f7      	beq.n	0x2c78
    2c88:	e776      	b.n	0x2b78
    2c8a:	4770      	bx	lr
    2c8c:	4770      	bx	lr
    2c8e:	46c0      	nop			@ (mov r8, r8)
    2c90:	b403      	push	{r0, r1}
    2c92:	4671      	mov	r1, lr
    2c94:	0849      	lsrs	r1, r1, #1
    2c96:	0040      	lsls	r0, r0, #1
    2c98:	0049      	lsls	r1, r1, #1
    2c9a:	5a09      	ldrh	r1, [r1, r0]
    2c9c:	0049      	lsls	r1, r1, #1
    2c9e:	448e      	add	lr, r1
    2ca0:	bc03      	pop	{r0, r1}
    2ca2:	4770      	bx	lr
    2ca4:	b510      	push	{r4, lr}
    2ca6:	4b03      	ldr	r3, [pc, #12]	@ (0x2cb4)
    2ca8:	0001      	movs	r1, r0
    2caa:	6818      	ldr	r0, [r3, #0]
    2cac:	f000 f830 	bl	0x2d10
    2cb0:	bd10      	pop	{r4, pc}
    2cb2:	46c0      	nop			@ (mov r8, r8)
    2cb4:	0020      	movs	r0, r4
    2cb6:	1000      	asrs	r0, r0, #32
    2cb8:	b510      	push	{r4, lr}
    2cba:	4b03      	ldr	r3, [pc, #12]	@ (0x2cc8)
    2cbc:	0001      	movs	r1, r0
    2cbe:	6818      	ldr	r0, [r3, #0]
    2cc0:	f000 f8f8 	bl	0x2eb4
    2cc4:	bd10      	pop	{r4, pc}
    2cc6:	46c0      	nop			@ (mov r8, r8)
    2cc8:	0020      	movs	r0, r4
    2cca:	1000      	asrs	r0, r0, #32
    2ccc:	b570      	push	{r4, r5, r6, lr}
    2cce:	4e0f      	ldr	r6, [pc, #60]	@ (0x2d0c)
    2cd0:	000d      	movs	r5, r1
    2cd2:	6831      	ldr	r1, [r6, #0]
    2cd4:	0004      	movs	r4, r0
    2cd6:	2900      	cmp	r1, #0
    2cd8:	d102      	bne.n	0x2ce0
    2cda:	f000 f8a9 	bl	0x2e30
    2cde:	6030      	str	r0, [r6, #0]
    2ce0:	0029      	movs	r1, r5
    2ce2:	0020      	movs	r0, r4
    2ce4:	f000 f8a4 	bl	0x2e30
    2ce8:	1c43      	adds	r3, r0, #1
    2cea:	d103      	bne.n	0x2cf4
    2cec:	2501      	movs	r5, #1
    2cee:	426d      	negs	r5, r5
    2cf0:	0028      	movs	r0, r5
    2cf2:	bd70      	pop	{r4, r5, r6, pc}
    2cf4:	2303      	movs	r3, #3
    2cf6:	1cc5      	adds	r5, r0, #3
    2cf8:	439d      	bics	r5, r3
    2cfa:	42a8      	cmp	r0, r5
    2cfc:	d0f8      	beq.n	0x2cf0
    2cfe:	1a29      	subs	r1, r5, r0
    2d00:	0020      	movs	r0, r4
    2d02:	f000 f895 	bl	0x2e30
    2d06:	3001      	adds	r0, #1
    2d08:	d1f2      	bne.n	0x2cf0
    2d0a:	e7ef      	b.n	0x2cec
    2d0c:	02b0      	lsls	r0, r6, #10
    2d0e:	1000      	asrs	r0, r0, #32
    2d10:	b5f7      	push	{r0, r1, r2, r4, r5, r6, r7, lr}
    2d12:	2203      	movs	r2, #3
    2d14:	1ccb      	adds	r3, r1, #3
    2d16:	4393      	bics	r3, r2
    2d18:	3308      	adds	r3, #8
    2d1a:	0005      	movs	r5, r0
    2d1c:	001f      	movs	r7, r3
    2d1e:	2b0c      	cmp	r3, #12
    2d20:	d234      	bcs.n	0x2d8c
    2d22:	270c      	movs	r7, #12
    2d24:	42b9      	cmp	r1, r7
    2d26:	d833      	bhi.n	0x2d90
    2d28:	0028      	movs	r0, r5
    2d2a:	f000 f871 	bl	0x2e10
    2d2e:	4e37      	ldr	r6, [pc, #220]	@ (0x2e0c)
    2d30:	6833      	ldr	r3, [r6, #0]
    2d32:	001c      	movs	r4, r3
    2d34:	2c00      	cmp	r4, #0
    2d36:	d12f      	bne.n	0x2d98
    2d38:	0039      	movs	r1, r7
    2d3a:	0028      	movs	r0, r5
    2d3c:	f7ff ffc6 	bl	0x2ccc
    2d40:	0004      	movs	r4, r0
    2d42:	1c43      	adds	r3, r0, #1
    2d44:	d15f      	bne.n	0x2e06
    2d46:	6834      	ldr	r4, [r6, #0]
    2d48:	9400      	str	r4, [sp, #0]
    2d4a:	9b00      	ldr	r3, [sp, #0]
    2d4c:	2b00      	cmp	r3, #0
    2d4e:	d14a      	bne.n	0x2de6
    2d50:	2c00      	cmp	r4, #0
    2d52:	d052      	beq.n	0x2dfa
    2d54:	6823      	ldr	r3, [r4, #0]
    2d56:	0028      	movs	r0, r5
    2d58:	18e3      	adds	r3, r4, r3
    2d5a:	9900      	ldr	r1, [sp, #0]
    2d5c:	9301      	str	r3, [sp, #4]
    2d5e:	f000 f867 	bl	0x2e30
    2d62:	9b01      	ldr	r3, [sp, #4]
    2d64:	4283      	cmp	r3, r0
    2d66:	d148      	bne.n	0x2dfa
    2d68:	6823      	ldr	r3, [r4, #0]
    2d6a:	0028      	movs	r0, r5
    2d6c:	1aff      	subs	r7, r7, r3
    2d6e:	0039      	movs	r1, r7
    2d70:	f7ff ffac 	bl	0x2ccc
    2d74:	3001      	adds	r0, #1
    2d76:	d040      	beq.n	0x2dfa
    2d78:	6823      	ldr	r3, [r4, #0]
    2d7a:	19db      	adds	r3, r3, r7
    2d7c:	6023      	str	r3, [r4, #0]
    2d7e:	6833      	ldr	r3, [r6, #0]
    2d80:	685a      	ldr	r2, [r3, #4]
    2d82:	2a00      	cmp	r2, #0
    2d84:	d133      	bne.n	0x2dee
    2d86:	9b00      	ldr	r3, [sp, #0]
    2d88:	6033      	str	r3, [r6, #0]
    2d8a:	e019      	b.n	0x2dc0
    2d8c:	2b00      	cmp	r3, #0
    2d8e:	dac9      	bge.n	0x2d24
    2d90:	230c      	movs	r3, #12
    2d92:	602b      	str	r3, [r5, #0]
    2d94:	2000      	movs	r0, #0
    2d96:	bdfe      	pop	{r1, r2, r3, r4, r5, r6, r7, pc}
    2d98:	6821      	ldr	r1, [r4, #0]
    2d9a:	1bc9      	subs	r1, r1, r7
    2d9c:	d420      	bmi.n	0x2de0
    2d9e:	290b      	cmp	r1, #11
    2da0:	d90a      	bls.n	0x2db8
    2da2:	19e2      	adds	r2, r4, r7
    2da4:	6027      	str	r7, [r4, #0]
    2da6:	42a3      	cmp	r3, r4
    2da8:	d104      	bne.n	0x2db4
    2daa:	6032      	str	r2, [r6, #0]
    2dac:	6863      	ldr	r3, [r4, #4]
    2dae:	6011      	str	r1, [r2, #0]
    2db0:	6053      	str	r3, [r2, #4]
    2db2:	e005      	b.n	0x2dc0
    2db4:	605a      	str	r2, [r3, #4]
    2db6:	e7f9      	b.n	0x2dac
    2db8:	6862      	ldr	r2, [r4, #4]
    2dba:	42a3      	cmp	r3, r4
    2dbc:	d10e      	bne.n	0x2ddc
    2dbe:	6032      	str	r2, [r6, #0]
    2dc0:	0028      	movs	r0, r5
    2dc2:	f000 f82d 	bl	0x2e20
    2dc6:	0020      	movs	r0, r4
    2dc8:	2207      	movs	r2, #7
    2dca:	300b      	adds	r0, #11
    2dcc:	1d23      	adds	r3, r4, #4
    2dce:	4390      	bics	r0, r2
    2dd0:	1ac2      	subs	r2, r0, r3
    2dd2:	4298      	cmp	r0, r3
    2dd4:	d0df      	beq.n	0x2d96
    2dd6:	1a1b      	subs	r3, r3, r0
    2dd8:	50a3      	str	r3, [r4, r2]
    2dda:	e7dc      	b.n	0x2d96
    2ddc:	605a      	str	r2, [r3, #4]
    2dde:	e7ef      	b.n	0x2dc0
    2de0:	0023      	movs	r3, r4
    2de2:	6864      	ldr	r4, [r4, #4]
    2de4:	e7a6      	b.n	0x2d34
    2de6:	9c00      	ldr	r4, [sp, #0]
    2de8:	6863      	ldr	r3, [r4, #4]
    2dea:	9300      	str	r3, [sp, #0]
    2dec:	e7ad      	b.n	0x2d4a
    2dee:	001a      	movs	r2, r3
    2df0:	685b      	ldr	r3, [r3, #4]
    2df2:	42a3      	cmp	r3, r4
    2df4:	d1fb      	bne.n	0x2dee
    2df6:	2300      	movs	r3, #0
    2df8:	e7da      	b.n	0x2db0
    2dfa:	230c      	movs	r3, #12
    2dfc:	0028      	movs	r0, r5
    2dfe:	602b      	str	r3, [r5, #0]
    2e00:	f000 f80e 	bl	0x2e20
    2e04:	e7c6      	b.n	0x2d94
    2e06:	6007      	str	r7, [r0, #0]
    2e08:	e7da      	b.n	0x2dc0
    2e0a:	46c0      	nop			@ (mov r8, r8)
    2e0c:	02b4      	lsls	r4, r6, #10
    2e0e:	1000      	asrs	r0, r0, #32
    2e10:	b510      	push	{r4, lr}
    2e12:	4802      	ldr	r0, [pc, #8]	@ (0x2e1c)
    2e14:	f000 f842 	bl	0x2e9c
    2e18:	bd10      	pop	{r4, pc}
    2e1a:	46c0      	nop			@ (mov r8, r8)
    2e1c:	02b8      	lsls	r0, r7, #10
    2e1e:	1000      	asrs	r0, r0, #32
    2e20:	b510      	push	{r4, lr}
    2e22:	4802      	ldr	r0, [pc, #8]	@ (0x2e2c)
    2e24:	f000 f83b 	bl	0x2e9e
    2e28:	bd10      	pop	{r4, pc}
    2e2a:	46c0      	nop			@ (mov r8, r8)
    2e2c:	02b8      	lsls	r0, r7, #10
    2e2e:	1000      	asrs	r0, r0, #32
    2e30:	2300      	movs	r3, #0
    2e32:	b570      	push	{r4, r5, r6, lr}
    2e34:	4d06      	ldr	r5, [pc, #24]	@ (0x2e50)
    2e36:	0004      	movs	r4, r0
    2e38:	0008      	movs	r0, r1
    2e3a:	602b      	str	r3, [r5, #0]
    2e3c:	f000 f8aa 	bl	0x2f94
    2e40:	1c43      	adds	r3, r0, #1
    2e42:	d103      	bne.n	0x2e4c
    2e44:	682b      	ldr	r3, [r5, #0]
    2e46:	2b00      	cmp	r3, #0
    2e48:	d000      	beq.n	0x2e4c
    2e4a:	6023      	str	r3, [r4, #0]
    2e4c:	bd70      	pop	{r4, r5, r6, pc}
    2e4e:	46c0      	nop			@ (mov r8, r8)
    2e50:	02bc      	lsls	r4, r7, #10
    2e52:	1000      	asrs	r0, r0, #32
    2e54:	b570      	push	{r4, r5, r6, lr}
    2e56:	2600      	movs	r6, #0
    2e58:	4c0c      	ldr	r4, [pc, #48]	@ (0x2e8c)
    2e5a:	4d0d      	ldr	r5, [pc, #52]	@ (0x2e90)
    2e5c:	1b64      	subs	r4, r4, r5
    2e5e:	10a4      	asrs	r4, r4, #2
    2e60:	42a6      	cmp	r6, r4
    2e62:	d109      	bne.n	0x2e78
    2e64:	2600      	movs	r6, #0
    2e66:	f000 f99d 	bl	0x31a4
    2e6a:	4c0a      	ldr	r4, [pc, #40]	@ (0x2e94)
    2e6c:	4d0a      	ldr	r5, [pc, #40]	@ (0x2e98)
    2e6e:	1b64      	subs	r4, r4, r5
    2e70:	10a4      	asrs	r4, r4, #2
    2e72:	42a6      	cmp	r6, r4
    2e74:	d105      	bne.n	0x2e82
    2e76:	bd70      	pop	{r4, r5, r6, pc}
    2e78:	00b3      	lsls	r3, r6, #2
    2e7a:	58eb      	ldr	r3, [r5, r3]
    2e7c:	4798      	blx	r3
    2e7e:	3601      	adds	r6, #1
    2e80:	e7ee      	b.n	0x2e60
    2e82:	00b3      	lsls	r3, r6, #2
    2e84:	58eb      	ldr	r3, [r5, r3]
    2e86:	4798      	blx	r3
    2e88:	3601      	adds	r6, #1
    2e8a:	e7f2      	b.n	0x2e72
    2e8c:	31b0      	adds	r1, #176	@ 0xb0
    2e8e:	0000      	movs	r0, r0
    2e90:	31b0      	adds	r1, #176	@ 0xb0
    2e92:	0000      	movs	r0, r0
    2e94:	31b8      	adds	r1, #184	@ 0xb8
    2e96:	0000      	movs	r0, r0
    2e98:	31b0      	adds	r1, #176	@ 0xb0
    2e9a:	0000      	movs	r0, r0
    2e9c:	4770      	bx	lr
    2e9e:	4770      	bx	lr
    2ea0:	2300      	movs	r3, #0
    2ea2:	b510      	push	{r4, lr}
    2ea4:	429a      	cmp	r2, r3
    2ea6:	d100      	bne.n	0x2eaa
    2ea8:	bd10      	pop	{r4, pc}
    2eaa:	5ccc      	ldrb	r4, [r1, r3]
    2eac:	54c4      	strb	r4, [r0, r3]
    2eae:	3301      	adds	r3, #1
    2eb0:	e7f8      	b.n	0x2ea4
    2eb2:	ffff b570 	vsli.32	<illegal reg q13.5>, q8, #31
    2eb6:	0005      	movs	r5, r0
    2eb8:	1e0c      	subs	r4, r1, #0
    2eba:	d010      	beq.n	0x2ede
    2ebc:	3c04      	subs	r4, #4
    2ebe:	6823      	ldr	r3, [r4, #0]
    2ec0:	2b00      	cmp	r3, #0
    2ec2:	da00      	bge.n	0x2ec6
    2ec4:	18e4      	adds	r4, r4, r3
    2ec6:	0028      	movs	r0, r5
    2ec8:	f7ff ffa2 	bl	0x2e10
    2ecc:	4a1d      	ldr	r2, [pc, #116]	@ (0x2f44)
    2ece:	6813      	ldr	r3, [r2, #0]
    2ed0:	2b00      	cmp	r3, #0
    2ed2:	d105      	bne.n	0x2ee0
    2ed4:	6063      	str	r3, [r4, #4]
    2ed6:	6014      	str	r4, [r2, #0]
    2ed8:	0028      	movs	r0, r5
    2eda:	f7ff ffa1 	bl	0x2e20
    2ede:	bd70      	pop	{r4, r5, r6, pc}
    2ee0:	42a3      	cmp	r3, r4
    2ee2:	d908      	bls.n	0x2ef6
    2ee4:	6820      	ldr	r0, [r4, #0]
    2ee6:	1821      	adds	r1, r4, r0
    2ee8:	428b      	cmp	r3, r1
    2eea:	d1f3      	bne.n	0x2ed4
    2eec:	6819      	ldr	r1, [r3, #0]
    2eee:	685b      	ldr	r3, [r3, #4]
    2ef0:	1809      	adds	r1, r1, r0
    2ef2:	6021      	str	r1, [r4, #0]
    2ef4:	e7ee      	b.n	0x2ed4
    2ef6:	001a      	movs	r2, r3
    2ef8:	685b      	ldr	r3, [r3, #4]
    2efa:	2b00      	cmp	r3, #0
    2efc:	d001      	beq.n	0x2f02
    2efe:	42a3      	cmp	r3, r4
    2f00:	d9f9      	bls.n	0x2ef6
    2f02:	6811      	ldr	r1, [r2, #0]
    2f04:	1850      	adds	r0, r2, r1
    2f06:	42a0      	cmp	r0, r4
    2f08:	d10b      	bne.n	0x2f22
    2f0a:	6820      	ldr	r0, [r4, #0]
    2f0c:	1809      	adds	r1, r1, r0
    2f0e:	1850      	adds	r0, r2, r1
    2f10:	6011      	str	r1, [r2, #0]
    2f12:	4283      	cmp	r3, r0
    2f14:	d1e0      	bne.n	0x2ed8
    2f16:	6818      	ldr	r0, [r3, #0]
    2f18:	685b      	ldr	r3, [r3, #4]
    2f1a:	1841      	adds	r1, r0, r1
    2f1c:	6011      	str	r1, [r2, #0]
    2f1e:	6053      	str	r3, [r2, #4]
    2f20:	e7da      	b.n	0x2ed8
    2f22:	42a0      	cmp	r0, r4
    2f24:	d902      	bls.n	0x2f2c
    2f26:	230c      	movs	r3, #12
    2f28:	602b      	str	r3, [r5, #0]
    2f2a:	e7d5      	b.n	0x2ed8
    2f2c:	6820      	ldr	r0, [r4, #0]
    2f2e:	1821      	adds	r1, r4, r0
    2f30:	428b      	cmp	r3, r1
    2f32:	d103      	bne.n	0x2f3c
    2f34:	6819      	ldr	r1, [r3, #0]
    2f36:	685b      	ldr	r3, [r3, #4]
    2f38:	1809      	adds	r1, r1, r0
    2f3a:	6021      	str	r1, [r4, #0]
    2f3c:	6063      	str	r3, [r4, #4]
    2f3e:	6054      	str	r4, [r2, #4]
    2f40:	e7ca      	b.n	0x2ed8
    2f42:	46c0      	nop			@ (mov r8, r8)
    2f44:	02b4      	lsls	r4, r6, #10
    2f46:	1000      	asrs	r0, r0, #32
    2f48:	4806      	ldr	r0, [pc, #24]	@ (0x2f64)
    2f4a:	4907      	ldr	r1, [pc, #28]	@ (0x2f68)
    2f4c:	1a09      	subs	r1, r1, r0
    2f4e:	108b      	asrs	r3, r1, #2
    2f50:	0fc9      	lsrs	r1, r1, #31
    2f52:	18c9      	adds	r1, r1, r3
    2f54:	b510      	push	{r4, lr}
    2f56:	1049      	asrs	r1, r1, #1
    2f58:	d003      	beq.n	0x2f62
    2f5a:	4b04      	ldr	r3, [pc, #16]	@ (0x2f6c)
    2f5c:	2b00      	cmp	r3, #0
    2f5e:	d000      	beq.n	0x2f62
    2f60:	4798      	blx	r3
    2f62:	bd10      	pop	{r4, pc}
    2f64:	0070      	lsls	r0, r6, #1
    2f66:	1000      	asrs	r0, r0, #32
    2f68:	0070      	lsls	r0, r6, #1
    2f6a:	1000      	asrs	r0, r0, #32
    2f6c:	0000      	movs	r0, r0
    2f6e:	0000      	movs	r0, r0
    2f70:	4b05      	ldr	r3, [pc, #20]	@ (0x2f88)
    2f72:	b510      	push	{r4, lr}
    2f74:	2b00      	cmp	r3, #0
    2f76:	d003      	beq.n	0x2f80
    2f78:	4904      	ldr	r1, [pc, #16]	@ (0x2f8c)
    2f7a:	4805      	ldr	r0, [pc, #20]	@ (0x2f90)
    2f7c:	e000      	b.n	0x2f80
    2f7e:	bf00      	nop
    2f80:	f7ff ffe2 	bl	0x2f48
    2f84:	bd10      	pop	{r4, pc}
    2f86:	46c0      	nop			@ (mov r8, r8)
    2f88:	0000      	movs	r0, r0
    2f8a:	0000      	movs	r0, r0
    2f8c:	02c0      	lsls	r0, r0, #11
    2f8e:	1000      	asrs	r0, r0, #32
    2f90:	31c4      	adds	r1, #196	@ 0xc4
    2f92:	0000      	movs	r0, r0
    2f94:	b570      	push	{r4, r5, r6, lr}
    2f96:	4d0d      	ldr	r5, [pc, #52]	@ (0x2fcc)
    2f98:	0004      	movs	r4, r0
    2f9a:	682b      	ldr	r3, [r5, #0]
    2f9c:	2b00      	cmp	r3, #0
    2f9e:	d101      	bne.n	0x2fa4
    2fa0:	4b0b      	ldr	r3, [pc, #44]	@ (0x2fd0)
    2fa2:	602b      	str	r3, [r5, #0]
    2fa4:	2303      	movs	r3, #3
    2fa6:	682e      	ldr	r6, [r5, #0]
    2fa8:	3403      	adds	r4, #3
    2faa:	439c      	bics	r4, r3
    2fac:	1934      	adds	r4, r6, r4
    2fae:	0020      	movs	r0, r4
    2fb0:	f000 f810 	bl	0x2fd4
    2fb4:	2800      	cmp	r0, #0
    2fb6:	d102      	bne.n	0x2fbe
    2fb8:	602c      	str	r4, [r5, #0]
    2fba:	0030      	movs	r0, r6
    2fbc:	bd70      	pop	{r4, r5, r6, pc}
    2fbe:	f000 f815 	bl	0x2fec
    2fc2:	2601      	movs	r6, #1
    2fc4:	230c      	movs	r3, #12
    2fc6:	4276      	negs	r6, r6
    2fc8:	6003      	str	r3, [r0, #0]
    2fca:	e7f6      	b.n	0x2fba
    2fcc:	02d8      	lsls	r0, r3, #11
    2fce:	1000      	asrs	r0, r0, #32
    2fd0:	02dc      	lsls	r4, r3, #11
    2fd2:	1000      	asrs	r0, r0, #32
    2fd4:	0003      	movs	r3, r0
    2fd6:	4804      	ldr	r0, [pc, #16]	@ (0x2fe8)
    2fd8:	2800      	cmp	r0, #0
    2fda:	d003      	beq.n	0x2fe4
    2fdc:	2200      	movs	r2, #0
    2fde:	4283      	cmp	r3, r0
    2fe0:	4152      	adcs	r2, r2
    2fe2:	0010      	movs	r0, r2
    2fe4:	4770      	bx	lr
    2fe6:	46c0      	nop			@ (mov r8, r8)
    2fe8:	0000      	movs	r0, r0
    2fea:	0000      	movs	r0, r0
    2fec:	4b01      	ldr	r3, [pc, #4]	@ (0x2ff4)
    2fee:	6818      	ldr	r0, [r3, #0]
    2ff0:	4770      	bx	lr
    2ff2:	46c0      	nop			@ (mov r8, r8)
    2ff4:	0020      	movs	r0, r4
    2ff6:	1000      	asrs	r0, r0, #32
    2ff8:	0000      	movs	r0, r0
    2ffa:	5000      	str	r0, [r0, r0]
    2ffc:	0000      	movs	r0, r0
    2ffe:	5001      	str	r1, [r0, r0]
    3000:	0000      	movs	r0, r0
    3002:	5002      	str	r2, [r0, r0]
    3004:	0000      	movs	r0, r0
    3006:	5003      	str	r3, [r0, r0]
    3008:	c226      	stmia	r2!, {r1, r2, r5}
    300a:	0033      	movs	r3, r6
    300c:	8247      	strh	r7, [r0, #18]
    300e:	0079      	lsls	r1, r7, #1
    3010:	c261      	stmia	r2!, {r0, r5, r6}
    3012:	0f34      	lsrs	r4, r6, #28
    3014:	c264      	stmia	r2!, {r2, r5, r6}
    3016:	0079      	lsls	r1, r7, #1
    3018:	0003      	movs	r3, r0
    301a:	0004      	movs	r4, r0
    301c:	0007      	movs	r7, r0
    301e:	000b      	movs	r3, r1
    3020:	000c      	movs	r4, r1
    3022:	000d      	movs	r5, r1
    3024:	0013      	movs	r3, r2
    3026:	0014      	movs	r4, r2
    3028:	0018      	movs	r0, r3
    302a:	0019      	movs	r1, r3
    302c:	001a      	movs	r2, r3
    302e:	001d      	movs	r5, r3
    3030:	001e      	movs	r6, r3
    3032:	001f      	movs	r7, r3
    3034:	0020      	movs	r0, r4
    3036:	0024      	movs	r4, r4
    3038:	0025      	movs	r5, r4
    303a:	0028      	movs	r0, r5
    303c:	0029      	movs	r1, r5
    303e:	002a      	movs	r2, r5
    3040:	0005      	movs	r5, r0
    3042:	000e      	movs	r6, r1
    3044:	001b      	movs	r3, r3
    3046:	0026      	movs	r6, r4
    3048:	0002      	movs	r2, r0
    304a:	000a      	movs	r2, r1
    304c:	0017      	movs	r7, r2
    304e:	0023      	movs	r3, r4
    3050:	0010      	movs	r0, r2
    3052:	0011      	movs	r1, r2
    3054:	0000      	movs	r0, r0
    3056:	0008      	movs	r0, r1
    3058:	0009      	movs	r1, r1
    305a:	0015      	movs	r5, r2
    305c:	0016      	movs	r6, r2
    305e:	001c      	movs	r4, r3
    3060:	0021      	movs	r1, r4
    3062:	0022      	movs	r2, r4
    3064:	0027      	movs	r7, r4
    3066:	002b      	movs	r3, r5
    3068:	000f      	movs	r7, r1
    306a:	0012      	movs	r2, r2
	...
    3080:	04eb      	lsls	r3, r5, #19
    3082:	0000      	movs	r0, r0
    3084:	04f5      	lsls	r5, r6, #19
    3086:	0000      	movs	r0, r0
    3088:	050b      	lsls	r3, r1, #20
    308a:	0000      	movs	r0, r0
    308c:	04bd      	lsls	r5, r7, #18
    308e:	0000      	movs	r0, r0
    3090:	04d1      	lsls	r1, r2, #19
	...
    30aa:	0000      	movs	r0, r0
    30ac:	04eb      	lsls	r3, r5, #19
    30ae:	0000      	movs	r0, r0
    30b0:	04f5      	lsls	r5, r6, #19
    30b2:	0000      	movs	r0, r0
    30b4:	050b      	lsls	r3, r1, #20
    30b6:	0000      	movs	r0, r0
    30b8:	04bd      	lsls	r5, r7, #18
    30ba:	0000      	movs	r0, r0
    30bc:	04d1      	lsls	r1, r2, #19
    30be:	0000      	movs	r0, r0
    30c0:	032d      	lsls	r5, r5, #12
    30c2:	0000      	movs	r0, r0
    30c4:	0335      	lsls	r5, r6, #12
    30c6:	0000      	movs	r0, r0
    30c8:	033d      	lsls	r5, r7, #12
    30ca:	0000      	movs	r0, r0
    30cc:	0341      	lsls	r1, r0, #13
	...
    30de:	0000      	movs	r0, r0
    30e0:	18d5      	adds	r5, r2, r3
    30e2:	0000      	movs	r0, r0
    30e4:	0345      	lsls	r5, r0, #13
    30e6:	0000      	movs	r0, r0
    30e8:	03b9      	lsls	r1, r7, #14
	...
    30f2:	0000      	movs	r0, r0
    30f4:	0359      	lsls	r1, r3, #13
	...
    30fe:	0000      	movs	r0, r0
    3100:	04b1      	lsls	r1, r6, #18
	...
    3112:	0000      	movs	r0, r0
    3114:	196d      	adds	r5, r5, r5
    3116:	0000      	movs	r0, r0
    3118:	0345      	lsls	r5, r0, #13
    311a:	0000      	movs	r0, r0
    311c:	15b5      	asrs	r5, r6, #22
	...
    3126:	0000      	movs	r0, r0
    3128:	035d      	lsls	r5, r3, #13
    312a:	0000      	movs	r0, r0
    312c:	0395      	lsls	r5, r2, #14
    312e:	0000      	movs	r0, r0
    3130:	06eb      	lsls	r3, r5, #27
    3132:	0000      	movs	r0, r0
    3134:	04b1      	lsls	r1, r6, #18
    3136:	0000      	movs	r0, r0
    3138:	0000      	movs	r0, r0
    313a:	0000      	movs	r0, r0
    313c:	0743      	lsls	r3, r0, #29
    313e:	0000      	movs	r0, r0
    3140:	0325      	lsls	r5, r4, #12
	...
    314a:	0000      	movs	r0, r0
    314c:	0329      	lsls	r1, r5, #12
    314e:	0000      	movs	r0, r0
    3150:	15a9      	asrs	r1, r5, #22
    3152:	0000      	movs	r0, r0
    3154:	196d      	adds	r5, r5, r5
    3156:	0000      	movs	r0, r0
    3158:	0345      	lsls	r5, r0, #13
    315a:	0000      	movs	r0, r0
    315c:	15b5      	asrs	r5, r6, #22
    315e:	0000      	movs	r0, r0
    3160:	0355      	lsls	r5, r2, #13
    3162:	0000      	movs	r0, r0
    3164:	0351      	lsls	r1, r2, #13
    3166:	0000      	movs	r0, r0
    3168:	034d      	lsls	r5, r1, #13
    316a:	0000      	movs	r0, r0
    316c:	0395      	lsls	r5, r2, #14
    316e:	0000      	movs	r0, r0
    3170:	06eb      	lsls	r3, r5, #27
    3172:	0000      	movs	r0, r0
    3174:	04b1      	lsls	r1, r6, #18
    3176:	0000      	movs	r0, r0
    3178:	0349      	lsls	r1, r1, #13
    317a:	0000      	movs	r0, r0
    317c:	0743      	lsls	r3, r0, #29
    317e:	0000      	movs	r0, r0
    3180:	0325      	lsls	r5, r4, #12
	...
    318a:	0000      	movs	r0, r0
    318c:	052b      	lsls	r3, r5, #20
    318e:	0000      	movs	r0, r0
    3190:	026d      	lsls	r5, r5, #9
    3192:	0000      	movs	r0, r0
    3194:	05b9      	lsls	r1, r7, #22
    3196:	0000      	movs	r0, r0
    3198:	0649      	lsls	r1, r1, #25
    319a:	0000      	movs	r0, r0
    319c:	065f      	lsls	r7, r3, #25
    319e:	0000      	movs	r0, r0
    31a0:	0259      	lsls	r1, r3, #9
    31a2:	0000      	movs	r0, r0
    31a4:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
    31a6:	46c0      	nop			@ (mov r8, r8)
    31a8:	bcf8      	pop	{r3, r4, r5, r6, r7}
    31aa:	bc08      	pop	{r3}
    31ac:	469e      	mov	lr, r3
    31ae:	4770      	bx	lr
    31b0:	2a53      	cmp	r2, #83	@ 0x53
    31b2:	0000      	movs	r0, r0
    31b4:	2f71      	cmp	r7, #113	@ 0x71
    31b6:	0000      	movs	r0, r0
    31b8:	b5f8      	push	{r3, r4, r5, r6, r7, lr}
    31ba:	46c0      	nop			@ (mov r8, r8)
    31bc:	bcf8      	pop	{r3, r4, r5, r6, r7}
    31be:	bc08      	pop	{r3}
    31c0:	469e      	mov	lr, r3
    31c2:	4770      	bx	lr
    31c4:	4121      	asrs	r1, r4
    31c6:	5056      	str	r6, [r2, r1]
    31c8:	4021      	ands	r1, r4
    31ca:	533a      	strh	r2, [r7, r4]
    31cc:	6642      	str	r2, [r0, #100]	@ 0x64
    31ce:	3174      	adds	r1, #116	@ 0x74
    31d0:	2032      	movs	r0, #50	@ 0x32
    31d2:	3020      	adds	r0, #32
    31d4:	302e      	adds	r0, #46	@ 0x2e
    31d6:	0032      	movs	r2, r6
    31d8:	8240      	strh	r0, [r0, #18]
    31da:	005c      	lsls	r4, r3, #1
    31dc:	8229      	strh	r1, [r5, #16]
    31de:	0069      	lsls	r1, r5, #1
    31e0:	6c00      	ldr	r0, [r0, #64]	@ 0x40
    31e2:	02dc      	lsls	r4, r3, #11
    31e4:	0024      	movs	r4, r4
    31e6:	1000      	asrs	r0, r0, #32
	...
    31f0:	0068      	lsls	r0, r5, #1
    31f2:	0000      	movs	r0, r0
    31f4:	00d0      	lsls	r0, r2, #3
	...
