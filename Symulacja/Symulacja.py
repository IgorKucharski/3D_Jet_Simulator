# Igor Kucharski, 2019

from panda3d.core import *
from panda3d.bullet import *
from direct.showbase.ShowBase import *
from direct.gui.OnscreenText import OnscreenText
import sys
import io
import numpy
from math import *
import time

loadPrcFileData('', 'win-size 1024 768')
loadPrcFileData('', 'win-origin 300 50')
loadPrcFileData('', 'window-title F-16')
loadPrcFileData('', 'show-frame-rate-meter true')


# STEROWANIE:
# F1 - włączanie / wyłączanie wyświetlania klawiszy;
# F2 - włączanie / wyłączanie osi współrzędnych układu odniesienia względem samolotu + wektora siły wypadkowej działającej na samolot;
# F3 - włączanie / wyłączenie debugera graficznego;
# TAB - przełączanie trybu osi współrzędnych samolotu - osie równoległe do osi globalnego układu / lokalnego układu samolotu;
# 0-5 - przełączanie różnych ujęć kamery;
# STRZAŁKA W GÓRĘ / DÓŁ - zwiększenie / zmniejszenie siły ciągu silnika (przyspieszanie / zwalnianie);
# STRZAŁKA W PRAWO / LEWO - zmniejszenie / zwiększenie kąta ustawienia steru kierunku (w prawo / w lewo);
# W/S - zmniejszenie / zwiększenie kąta ustawienia steru wysokości (w górę / w dół);
# A/D - zmniejszenie / zwiększenie kąta ustawienia lotek (obrót w lewo / w prawo);


class App(ShowBase, BulletWorld):
	"""Główna klasa aplikacji."""

	# inicjalizacja klasy;
	def __init__(self):			
		# deklaracja, że klasa App dziedziczy z klasy ShowBase;						
		ShowBase.__init__(self)		
		# deklaracja, że klasa App dziedziczy z klasy BulletWorld;					
		BulletWorld.__init__(self)		

		# utworzenie środowiska fizycznego self.world za pomocą klasy BulletWorld;
		self.world = BulletWorld()
		# ustawienie wektora przyspieszenia grawitacyjnego w środowisku self.world 
		# 	na wartość 9,81 m/s^2 skierowaną pionowo w dół;						
		self.world.setGravity(Vec3(0, 0, -9.81))		

		self.render = render
		self.render.setShaderAuto()
		self.airDensity = 1.225
		base.disableMouse()
		base.cam.setPos(75, 75, 75)
		base.cam.lookAt(0, 0, 0)

		skyColor = VBase3(135 / 255.0, 206 / 255.0, 235 / 255.0)
		base.set_background_color(skyColor)			

		self.fog = Fog('distanceFog')
		self.fog.setColor(0.6, 0.6, 0.6)
		self.fog.setExpDensity(.0001)
		self.render.setFog(self.fog)

		self.terrain = Terrain(self)
		self.jet = Jet(self)
		self.light = Light(self)
		self.cs = LocalCoordinateSystem(self, self.jet, 10)
		
		self.debug = Debug(self)

		self.time = 0
		self.cameraPos = 2
		self.showTexts = False

		self.taskMgr.add(self.update, 'update')	
		self.keyboardSetup()
		globalClock.setFrameRate(50)
		globalClock.setMode(5)


		self.showSteeringText = self.genLabelText(1, "F1: Pokaz sterowanie")
		# self.showSteeringTexts()


	def genLabelText(self, i, text):
		return OnscreenText(text=text, parent=base.a2dTopLeft, scale=.05,
							pos=(0.06, -.065 * i), fg=(1, 1, 1, 1),
							align=TextNode.ALeft)

	def showSteeringTexts(self):
		if self.showTexts:			
			self.showSteeringText.setText("F1 - Ukryj sterowanie")
			self.showCoordsText = self.genLabelText(2, "F2 - Pokaz / ukryj osie i wektor sily wypadkowej")
			self.toggleCoordsText = self.genLabelText(3, "TAB - Zmiana trybu wyswietlania osi")
			self.showDebugText = self.genLabelText(4, "F3 - Wlacz / wylacz debuger graficzny")
			self.showCamText = self.genLabelText(5, "[0]-[5] - Przelaczanie roznych ujec kamery")
			self.showArrowUpDownText = self.genLabelText(6, "STRZALKA W GORE / DOL - zwiekszenie / zmniejszenie sily ciagu silnika (przyspieszanie / zwalnianie)")
			self.showArrowLeftRightText = self.genLabelText(7, "STRZALKA W PRAWO / LEWO - zmniejszenie / zwiekszenie kata ustawienia steru kierunku (w prawo / w lewo)")
			self.showWSText = self.genLabelText(8, "[W]/[S] - zmniejszenie / zwiekszenie kata ustawienia steru wysokosci (w gore / w dol)")
			self.showADText = self.genLabelText(9, "[A]/[D] - zmniejszenie / zwiekszenie kata ustawienia lotek (obrot w lewo / w prawo)")
					
		else:
			self.showSteeringText.setText("F1 - Pokaz sterowanie")
			self.showCoordsText.cleanup()
			self.toggleCoordsText.cleanup()
			self.showDebugText.cleanup()
			self.showCamText.cleanup()
			self.showArrowUpDownText.cleanup()
			self.showArrowLeftRightText.cleanup()
			self.showWSText.cleanup()
			self.showADText.cleanup()


	def toggleSteeringTextsDisplay(self):
		if self.showTexts:
			self.showTexts = False
		else:
			self.showTexts = True

		self.showSteeringTexts()


	def keyboardSetup(self):
		self.keyMap = {"powerUp": 0, "powerDown": 0, "upward": 0, "downward": 0, "turnRight": 0, "turnLeft": 0, "rollRight": 0, "rollLeft": 0}

		self.accept("escape", sys.exit)

		self.accept("w", self.setKey, ["downward",1])
		self.accept("w-up", self.setKey, ["downward",0])
		self.accept("s", self.setKey, ["upward",1])
		self.accept("s-up", self.setKey, ["upward",0])

		self.accept("d", self.setKey, ["rollLeft",1])
		self.accept("d-up", self.setKey, ["rollLeft",0])
		self.accept("a", self.setKey, ["rollRight",1])
		self.accept("a-up", self.setKey, ["rollRight",0])

		self.accept("arrow_up", self.setKey, ["powerUp",1])
		self.accept("arrow_up-up", self.setKey, ["powerUp",0])
		self.accept("arrow_down", self.setKey, ["powerDown",1])
		self.accept("arrow_down-up", self.setKey, ["powerDown",0])

		self.accept("arrow_right", self.setKey, ["turnRight",1])
		self.accept("arrow_right-up", self.setKey, ["turnRight",0])
		self.accept("arrow_left", self.setKey, ["turnLeft",1])
		self.accept("arrow_left-up", self.setKey, ["turnLeft",0])

		self.accept("f1", self.toggleSteeringTextsDisplay)
		self.accept("f2", self.cs.toggleCS)
		self.accept("f3", self.debug.toggleDebug)
		self.accept("tab", self.cs.switchCS)

		self.accept("0", self.cameraPosChange, [0])
		self.accept("1", self.cameraPosChange, [1])
		self.accept("2", self.cameraPosChange, [2])
		self.accept("3", self.cameraPosChange, [3])
		self.accept("4", self.cameraPosChange, [4])
		self.accept("5", self.cameraPosChange, [5])


	def setKey(self, key, value):
		self.keyMap[key] = value


	def cameraPosChange(self, posNr):
		if posNr == 0:
			self.cameraPos = 0
		elif posNr == 1:
			self.cameraPos = 1
		elif posNr == 2:
			self.cameraPos = 2
		elif posNr == 3:
			self.cameraPos = 3
		elif posNr == 4:
			self.cameraPos = 4
		elif posNr == 5:
			self.cameraPos = 5


	def cameraUpdate(self):
		base.cam.lookAt(self.jet.jetNP.getPos())
		if self.cameraPos == 0:
			base.cam.setPos(20, 20, 15)
		elif self.cameraPos == 1:
			base.cam.setPos(self.jet.camPos_1)
		elif self.cameraPos == 2:
			base.cam.setPos(self.jet.camPos_2)
		elif self.cameraPos == 3:
			base.cam.setPos(self.jet.camPos_3)
		elif self.cameraPos == 4:
			base.cam.setPos(self.jet.camPos_4)
		elif self.cameraPos == 5:
			base.cam.setPos(self.jet.camPos_5)	


	def update(self, task):		
		self.cameraUpdate()
		dt = globalClock.getDt()
		self.world.doPhysics(dt, 10, 0.01)
		self.jet.update()
		self.cs.update()
		self.light.update()

		return task.cont



class Terrain():
	"""EN: Class which defines terrain.
	   PL: Klasa definiująca teren."""

	def __init__(self, environment):
		self.environment = environment
		self.shape = BulletPlaneShape(Vec3(0, 0, 1), 1)
		self.node = BulletRigidBodyNode('Ground')
		self.node.addShape(self.shape)
		self.np = self.environment.render.attachNewNode(self.node)
		self.np.setPos(0, 0, 0)
		self.environment.world.attach(self.node)

		floorTex = loader.loadTexture('maps/envir-ground.jpg')
		floor = self.environment.render.attachNewNode(PandaNode("floor"))

		self.cm = CardMaker("ground")
		self.cm.setFrame(-20000, 20000, -20000, 20000)
		self.ground = floor.attachNewNode(self.cm.generate())
		self.ground.setPos(0, 0, 1)
		self.ground.setHpr(0,-90,0)
		floor.setTexture(floorTex)
		floor.flattenStrong()
		self.node.setFriction(1)
		self.np.setCollideMask(BitMask32.allOn())



class Jet():
	"""Klasa definiująca model odrzutowca."""

	# Inicjalizacja klasy;
	def __init__(self, environment):
		# Zdefiniowanie środowiska, w tym przypadku utworzony obiekt 
		#	wykorzystuje środowisko self.world z klasy App;
		self.environment = environment		

		# Wywołanie funkcji zawierającej zbiór parametrów samolotu;
		self.jetParameters()
		self.jetColor = Vec4(.35, .55, .65, 1)

		# JET

		# Wczytanie uprzednio stworzonego modelu 3D;
		self.body = loader.loadModel("F-16_body.bam")
		self.body.flattenStrong()
		self.bodyMesh = BulletTriangleMesh()
		for geomNP in self.body.findAllMatches('**/+GeomNode'):
			geomNode = geomNP.node()
			ts = geomNP.getTransform(self.body)
			for geom in geomNode.getGeoms():
				self.bodyMesh.addGeom(geom, ts)
		# Przypisanie wczytanego modelu do zmiennej self.bodyShape;
		self.bodyShape = BulletTriangleMeshShape(self.bodyMesh, True)

		# Utworzenie punktu węzłowego self.jetNP o charakterystyce ciała sztywnego;
		self.jetNP = self.environment.render.attachNewNode(BulletRigidBodyNode('Jet'))
		self.jetNP.setPos(0, 0, 2.88)
		self.jetNP.setHpr(0, 0, 0)
		self.jetNP.setColor(self.jetColor)
		self.jetNP.node().setDeactivationEnabled(False)
		# Przypisanie wczytanego do zmiennej self.bodyShape modelu 
		#	do punktu węzłowego self.jetNP;
		self.jetNP.node().addShape(self.bodyShape)
		# Przypisanie masy zapisanej w zmiennej self.mass
		#	do punktu self.jetNP, reprezentującego samolot;
		self.jetNP.node().setMass(self.mass)							
		self.jetNP.node().setFriction(1)
		self.jetNP.node().setAngularDamping(0.95)
		self.jetNP.setShaderAuto()

		# ELEVATOR

		self.bodyElevator = loader.loadModel("F-16_elevator.bam")
		self.bodyElevator.flattenStrong()
		self.bodyElevatorMesh = BulletTriangleMesh()
		for geomNP in self.bodyElevator.findAllMatches('**/+GeomNode'):
			geomNode = geomNP.node()
			ts = geomNP.getTransform(self.bodyElevator)
			for geom in geomNode.getGeoms():
				self.bodyElevatorMesh.addGeom(geom, ts)
		self.bodyElevatorShape = BulletTriangleMeshShape(self.bodyElevatorMesh, True)

		self.jetElevatorNP = self.environment.render.attachNewNode(BulletRigidBodyNode('Elevator'))
		self.jetElevatorNP.setPos(self.jetNP.getPos())
		self.jetElevatorNP.setHpr(0, 0, 0)
		self.jetElevatorNP.setColor(self.jetColor)
		self.jetElevatorNP.node().setDeactivationEnabled(False)
		self.jetElevatorNP.node().addShape(self.bodyElevatorShape)
		self.jetElevatorNP.node().setMass(1)							
		self.jetElevatorNP.node().setAngularDamping(0.95)
		self.jetElevatorNP.setShaderAuto()
		self.jetElevatorNP.setCollideMask(BitMask32.allOff())

		# RUDDER

		self.bodyRudder = loader.loadModel("F-16_rudder.bam")
		self.bodyRudder.flattenStrong()
		self.bodyRudderMesh = BulletTriangleMesh()
		for geomNP in self.bodyRudder.findAllMatches('**/+GeomNode'):
			geomNode = geomNP.node()
			ts = geomNP.getTransform(self.bodyRudder)
			for geom in geomNode.getGeoms():
				self.bodyRudderMesh.addGeom(geom, ts)
		self.bodyRudderShape = BulletTriangleMeshShape(self.bodyRudderMesh, True)

		self.jetRudderNP = self.environment.render.attachNewNode(BulletRigidBodyNode('Rudder'))
		self.jetRudderNP.setPos(self.jetNP.getPos())
		self.jetRudderNP.setHpr(0, 0, 0)
		self.jetRudderNP.setColor(self.jetColor)
		self.jetRudderNP.node().setDeactivationEnabled(False)
		self.jetRudderNP.node().addShape(self.bodyRudderShape)
		self.jetRudderNP.node().setMass(1)							
		self.jetRudderNP.node().setAngularDamping(0.95)
		self.jetRudderNP.setShaderAuto()
		self.jetRudderNP.setCollideMask(BitMask32.allOff())

		# LEFT AILERON

		self.bodyLeftAileron = loader.loadModel("F-16_aileron_left.bam")
		self.bodyLeftAileron.flattenStrong()
		self.bodyLeftAileronMesh = BulletTriangleMesh()
		for geomNP in self.bodyLeftAileron.findAllMatches('**/+GeomNode'):
			geomNode = geomNP.node()
			ts = geomNP.getTransform(self.bodyLeftAileron)
			for geom in geomNode.getGeoms():
				self.bodyLeftAileronMesh.addGeom(geom, ts)
		self.bodyLeftAileronShape = BulletTriangleMeshShape(self.bodyLeftAileronMesh, True)

		self.jetLeftAileronNP = self.environment.render.attachNewNode(BulletRigidBodyNode('LeftAileron'))
		self.jetLeftAileronNP.setPos(self.jetNP.getPos())
		self.jetLeftAileronNP.setHpr(0, 0, 0)
		self.jetLeftAileronNP.setColor(self.jetColor)
		self.jetLeftAileronNP.node().setDeactivationEnabled(False)
		self.jetLeftAileronNP.node().addShape(self.bodyLeftAileronShape)
		self.jetLeftAileronNP.node().setMass(1)							
		self.jetLeftAileronNP.node().setAngularDamping(0.95)
		self.jetLeftAileronNP.setShaderAuto()
		self.jetLeftAileronNP.setCollideMask(BitMask32.allOff())

		# RIGHT AILERON

		self.bodyRightAileron = loader.loadModel("F-16_aileron_right.bam")
		self.bodyRightAileron.flattenStrong()
		self.bodyRightAileronMesh = BulletTriangleMesh()
		for geomNP in self.bodyRightAileron.findAllMatches('**/+GeomNode'):
			geomNode = geomNP.node()
			ts = geomNP.getTransform(self.bodyRightAileron)
			for geom in geomNode.getGeoms():
				self.bodyRightAileronMesh.addGeom(geom, ts)
		self.bodyRightAileronShape = BulletTriangleMeshShape(self.bodyRightAileronMesh, True)

		self.jetRightAileronNP = self.environment.render.attachNewNode(BulletRigidBodyNode('RightAileron'))
		self.jetRightAileronNP.setPos(self.jetNP.getPos())
		self.jetRightAileronNP.setHpr(0, 0, 0)
		self.jetRightAileronNP.setColor(self.jetColor)
		self.jetRightAileronNP.node().setDeactivationEnabled(False)
		self.jetRightAileronNP.node().addShape(self.bodyRightAileronShape)
		self.jetRightAileronNP.node().setMass(1)							
		self.jetRightAileronNP.node().setAngularDamping(0.95)
		self.jetRightAileronNP.setShaderAuto()
		self.jetRightAileronNP.setCollideMask(BitMask32.allOff())


		self.jetFrontNP = self.environment.render.attachNewNode("JetFrontNP")
		self.jetFrontNP.setPos(Point3(0.5 * self.jetLength, 0, 0))
		self.jetFrontNP.setHpr(0, 0, 0)
		self.jetFrontPos = self.jetFrontNP.getPos(self.environment.render)

		self.jetLeftWingEdgeNP = self.environment.render.attachNewNode("JetLeftWingEdgeNP")
		self.jetLeftWingEdgeNP.setPos(Point3(0, 0.5 * self.wingsSpan, 0))
		self.jetLeftWingEdgeNP.setHpr(0, 0, 0)
		self.jetLeftWingEdgePos = self.jetLeftWingEdgeNP.getPos(self.environment.render)

		self.jetRightWingEdgeNP = self.environment.render.attachNewNode("JetRightWingEdgeNP")
		self.jetRightWingEdgeNP.setPos(Point3(0, -0.5 * self.wingsSpan, 0))
		self.jetRightWingEdgeNP.setHpr(0, 0, 0)
		self.jetRightWingEdgePos = self.jetRightWingEdgeNP.getPos(self.environment.render)

		self.jetRearNP = self.environment.render.attachNewNode("JetRearNP")
		self.jetRearNP.setPos(Point3(-0.5 * self.jetLength, 0, 0))
		self.jetRearNP.setHpr(0, 0, 0)
	
		self.jetCenterAbsNP = self.environment.render.attachNewNode("JetCenterAbsNP")
		self.jetCenterAbsNP.setPos(self.jetNP.getPos())
		self.jetCenterAbsNP.setHpr(0, 0, 0)
	
		self.environment.world.attach(self.jetNP.node())
		self.environment.world.attach(self.jetElevatorNP.node())
		self.environment.world.attach(self.jetRudderNP.node())
		self.environment.world.attach(self.jetLeftAileronNP.node())
		self.environment.world.attach(self.jetRightAileronNP.node())
		
		self.body.reparentTo(self.jetNP)
		self.bodyElevator.reparentTo(self.jetElevatorNP)
		self.bodyRudder.reparentTo(self.jetRudderNP)
		self.bodyLeftAileron.reparentTo(self.jetLeftAileronNP)
		self.bodyRightAileron.reparentTo(self.jetRightAileronNP)

		self.jetFrontNP.reparentTo(self.jetNP)
		self.jetLeftWingEdgeNP.reparentTo(self.jetNP)
		self.jetRightWingEdgeNP.reparentTo(self.jetNP)
		self.jetRearNP.reparentTo(self.jetNP)		
		self.jetCenterAbsNP.reparentTo(self.environment.render)

		self.vehicle = BulletVehicle(self.environment.world, self.jetNP.node())
		self.environment.world.attach(self.vehicle)

		wheel_1_NP = loader.loadModel('yugotireR.egg')
		wheel_1_NP.reparentTo(self.environment.render)
		self.wheel(Point3(2, 0, -1.3), True, wheel_1_NP)

		wheel_2_NP = loader.loadModel('yugotireR.egg')
		wheel_2_NP.reparentTo(self.environment.render)
		self.wheel(Point3(-2, 2, -1.25), False, wheel_2_NP)

		wheel_3_NP = loader.loadModel('yugotireR.egg')
		wheel_3_NP.reparentTo(self.environment.render)
		self.wheel(Point3(-2, -2, -1.25), False, wheel_3_NP)

		self.lookAtPos = self.jetNP.getPos()
		self.camPos = self.jetNP.getPos() + Point3(0, 0, 30)
		self.camPos_2 = self.jetNP.getPos() + Point3(-40, 0, 5)





	def wheel(self, connectionPoint, frontWheel, nodePoint):

		wheel = self.vehicle.createWheel()
		 
		wheel.setNode(nodePoint.node())
		wheel.setChassisConnectionPointCs(connectionPoint)
		wheel.setFrontWheel(frontWheel)
		 
		wheel.setWheelDirectionCs(Vec3(0, 0, -1))
		wheel.setWheelAxleCs(Vec3(0, 1, 0))
		wheel.setWheelRadius(0.25)
		wheel.setMaxSuspensionTravelCm(40.0)
		 
		wheel.setSuspensionStiffness(40.0)
		wheel.setWheelsDampingRelaxation(2.3)
		wheel.setWheelsDampingCompression(4.4)
		wheel.setFrictionSlip(100.0)
		wheel.setRollInfluence(0.1)

		wheel.setMaxSuspensionForce(200000)


	def jetParameters(self):

		# STAŁE:

		self.mass = 8500										# masa [kg];

		self.minThrust = 0										# minimalny ciąg silnika samolotu [N];
		self.maxThrust = 129000									# maksymalny ciąg silnika samolotu [N];
		
		self.leftWingStaticArea = 8.85 							# powierzchnia lewego skrzydła (bez lotek) [m^2];
		self.rightWingStaticArea = 8.85 						# powierzchnia prawego skrzydła (bez lotek) [m^2];
		self.wingsStaticArea = self.leftWingStaticArea \
						+ self.rightWingStaticArea
		self.leftAileronArea = 1.15						# powierzchnia lewej lotki [m^2];
		self.rightAileronArea = 1.15					# powierzchnia prawej lotki [m^2];
		self.verticalStabilizerArea = 4.20
		self.rudderArea = 1.40

		self.minElevatorsAngle = -25							# minimalny kąt ustawienia statecznika poziomego (steru wysokości) [deg];
		self.maxElevatorsAngle = 25								# maksymalny kąt ustawienia statecznika poziomego (steru wysokości) [deg];
		self.minAileronAngle = -21.5								# minimalny kąt ustawienia lotki [deg];
		self.maxAileronAngle = 21.5								# maksymalny kąt ustawienia lotki [deg];
		self.minRudderAngle = -30								# minimalny kąt ustawienia steru kierunku [deg];
		self.maxRudderAngle = 30								# maksymalny kąt ustawienia steru kierunku [deg];
		self.leftElevatorArea = 5.9						# powierzchnia lewego statecznika poziomego (steru wysokości) [m^2];
		self.rightElevatorArea = 5.9					# powierzchnia prawego statecznika poziomego (steru wysokości) [m^2];
		self.elevatorsArea = self.leftElevatorArea \
						+ self.rightElevatorArea
		self.wingsArea = self.leftWingStaticArea \
					+ self.rightWingStaticArea \
					+ self.leftElevatorArea \
					+ self.rightElevatorArea					# powierzchnia skrzydeł [m^2];

		self.jetLength = 15.2							# długość samolotu [m];
		self.wingsSpan = 9.524 							# rozpiętość skrzydeł [m];
		self.volume = 24.5425							# objętość samolotu [m^3];
		self.buoyancy = self.volume \
					* self.environment.airDensity \
					* -self.environment.world.getGravity()[2]	# wartość siły wyporu powietrza [N];
		self.buoyancyForce = Vec3(0, 0, self.buoyancy)			# wektor siły wyporu o zwrocie przeciwnym niż wektor przyspieszenia grawitacyjnego;

		self.Cl_coeff = 0.042 									# wzrost współczynnika siły nośnej na każdy 1 stopień wzrostu kąta natarcia;
		self.Cl_coeff_max = 1.9 								# maksymalny współczynnik siły nośnej dla 30 stopni kąta natarcia;
		self.maxAngleForCoeff = 45								# maksymalny kąt natarcia dla liniowej wartości współczynnika;

		self.baseDragCoeff = 0.0175								# bazowy współczynnik oporu aerodynamicznego; 
		self.bottomDragCoeff = 1								# współczynnik oporu aerodynamicznego od spodu (dla powierzchni płaskiej);

		self.aspectRatio = self.wingsSpan**2 / self.wingsArea 	# współczynnik wydłużenia lambda;
		self.factor_e = 0.9084									# mnożnik e;

		self.wingsToElevatorsRatio = self.wingsStaticArea / self.elevatorsArea

		# ZMIENNE FIZYCZNE:

		self.thrust = 0											# siła ciągu silników [N];
		self.liftForceVal1e = 0									# siła nośna [N];
		self.dragForceValue = 0									# siła oporu aerodynamicznego [N];

		self.aileronAngle = 0
		self.leftAileronAngle = 0								# kąt natarcia prawej lotki [deg];
		self.rightAileronAngle = 0								# kąt natarcia lewej lotki [deg];
		
		self.elevatorsAngle = 0									# kąt natarcia statecznika poziomego (steru wysokości) [deg];
		self.leftElevatorAngle = self.elevatorsAngle 			# kąt natarcia lewego statecznika poziomego (steru wysokości) [deg];
		self.rightElevatorAngle = self.elevatorsAngle 			# kąt natarcia prawego statecznika poziomego (steru wysokości) [deg];
		self.rudderAngle = 0									# kąt natarcia statecznika pionowego (steru kierunku) [deg];

		self.velocityValue = 0									# wartość prędkości w kierunku ruchu (długość wektora) [m/s];
		self.pitchAngle = 0									# warość kąta natarcia samolotu [deg];

		# ZMIENNE MATEMATYCZNE:

		self.jetCenterMatrix = LMatrix4f()						# deklaracja macierzy 4D definiującej położenie i obroty centralnego punktu odrzutowca;
		self.jetFrontVector = Vec3()
		self.jetVelocityVectorNorm = Vec3()
		self.jetUpVectorNorm = Vec3()
		self.jetLiftVector = Vec3()
		self.jetDragVector = Vec3()
		self.elevatorsDragVector = Vec3()
		self.jetElevatorsVector = Vec3()
		self.jetElevatorsLiftVector = Vec3()

		# WEKTORY 

		self.lines = [DrawVector(self.environment, self, 10, vectorColor = Vec4(255%(i+1),255%(i+2),255%(i+3),1)) for i in range(10)]

		# TEKSTY WYŚWIETLANE NA EKRANIE

		self.speedText = OnscreenText(text="Predkosc: 0 m/s", parent=base.a2dBottomLeft, scale=.05,
							pos=(0.06, .260), fg=(1, 1, 1, 1),
							align=TextNode.ALeft)

		self.thrustText = OnscreenText(text="Sila ciagu silnika: 0 N", parent=base.a2dBottomLeft, scale=.05,
							pos=(0.06, .195), fg=(1, 1, 1, 1),
							align=TextNode.ALeft)

		self.attitudeText = OnscreenText(text="Wysokosc: 0 m", parent=base.a2dBottomLeft, scale=.05,
							pos=(0.06, .130), fg=(1, 1, 1, 1),
							align=TextNode.ALeft)

		self.angleText = OnscreenText(text="Kat natarcia: 0 "+chr(176), parent=base.a2dBottomLeft, scale=.05,
							pos=(0.06, .065), fg=(1, 1, 1, 1),
							align=TextNode.ALeft)


	def changeSetting(self, parameter, decrease, increase, minVal, maxVal, coeff = 1, relaxation = True, zeroVal = 0):

		jump = False

		if self.environment.keyMap[increase] and not self.environment.keyMap[decrease]:
			if parameter < maxVal:
				parameter += coeff
			else:
				parameter = parameter
		elif self.environment.keyMap[decrease] and not self.environment.keyMap[increase]:
			if parameter > minVal:
				parameter -= coeff
			else:
				parameter = parameter		
		elif not self.environment.keyMap[increase] and not self.environment.keyMap[decrease]:
			if relaxation:
				jump = True
			else:
				jump = False
		elif self.environment.keyMap[increase] and self.environment.keyMap[decrease]:
			if not relaxation:
				jump = True
			else:
				jump = False
		else:
			pass

		if jump:
			if parameter < zeroVal:
				parameter += coeff
			elif parameter > zeroVal:
				parameter -= coeff
			elif parameter == zeroVal:
				self.environment.keyMap[decrease] = 0
				self.environment.keyMap[increase] = 0
		else:
			pass

		return parameter


	def keysUpdate(self):
	
		self.thrust = self.changeSetting(self.thrust, "powerDown", "powerUp", self.minThrust, self.maxThrust, coeff = 1000, relaxation = False, zeroVal = 50000)
		self.rudderAngle = self.changeSetting(self.rudderAngle, "turnLeft", "turnRight", self.minRudderAngle, self.maxRudderAngle)
		self.elevatorsAngle = self.changeSetting(self.elevatorsAngle, "downward", "upward", self.minElevatorsAngle, self.maxElevatorsAngle)
		self.aileronAngle = self.changeSetting(self.aileronAngle, "rollLeft", "rollRight", self.minAileronAngle, self.maxAileronAngle)
		self.rightAileronAngle = self.aileronAngle
		self.leftAileronAngle = -self.aileronAngle

		print("Thrust: ", self.thrust)
		print("Elevator angle setting: ", self.elevatorsAngle)
		print("Aileron angle setting: ", self.aileronAngle)
		print("Rudder angle setting: ", self.rudderAngle)
		print("")


	def liftCoeffCalculate(self, angle):
		"""Funkcja służąca do obliczania współczynnika siły nośnej"""

		return round(1.9 * sin(2 * radians(angle) + 0.2), 2)


	def rudderCoeffCalculate(self, angle):
		return round(1.9 * sin(2 * radians(angle)), 2)


	def dragCoeffCalculate(self, angle):
		"""Funkcja służąca do obliczania współczynnika siły oporu"""

		return round(0.001 * angle**2 + 0.01, 2)


	def flapsSettings(self):

		# ELEVATORS

		point = 6.5
		vec1 = self.jetCenterMatrix.xformPoint(Point3(-point+point*cos(radians(self.elevatorsAngle)), 0, point*sin(radians(self.elevatorsAngle))))
		mat1 = numpy.array(self.jetCenterMatrix)
		mat2 = self.elevatorsRotationMatrix
		mat2 = numpy.array(mat2)
		mat3 = mat1.dot(mat2)
		mat3[3][0] = vec1[0]
		mat3[3][1] = vec1[1]
		mat3[3][2] = vec1[2]
		mat4 = Mat4(mat3[0][0], mat3[0][1], mat3[0][2], mat3[0][3], \
			mat3[1][0], mat3[1][1], mat3[1][2], mat3[1][3], \
			mat3[2][0], mat3[2][1], mat3[2][2], mat3[2][3], \
			mat3[3][0], mat3[3][1], mat3[3][2], mat3[3][3])
		self.jetElevatorNP.setMat(mat4)

		# RUDDER

		point = 6.5
		vec1 = self.jetCenterMatrix.xformPoint(Point3(-point+point*cos(radians(self.rudderAngle)), point*sin(radians(self.rudderAngle)), 0))
		# vec1 = self.jetCenterMatrix.xformPoint(Point3(0, 0, 0))
		mat1 = numpy.array(self.jetCenterMatrix)
		mat2 = self.rudderRotationMatrix
		mat2 = numpy.array(mat2)
		mat3 = mat1.dot(mat2)
		mat3[3][0] = vec1[0]
		mat3[3][1] = vec1[1]
		mat3[3][2] = vec1[2]
		mat4 = Mat4(mat3[0][0], mat3[0][1], mat3[0][2], mat3[0][3], \
			mat3[1][0], mat3[1][1], mat3[1][2], mat3[1][3], \
			mat3[2][0], mat3[2][1], mat3[2][2], mat3[2][3], \
			mat3[3][0], mat3[3][1], mat3[3][2], mat3[3][3])
		self.jetRudderNP.setMat(mat4)

		# LEFT AILERON

		point = 3.4
		vec1 = self.jetCenterMatrix.xformPoint(Point3(-point+point*cos(radians(self.leftAileronAngle)), 0, point*sin(radians(self.leftAileronAngle))))
		mat1 = numpy.array(self.jetCenterMatrix)
		mat2 = self.leftAileronRotationMatrix
		mat2 = numpy.array(mat2)
		mat3 = mat1.dot(mat2)
		mat3[3][0] = vec1[0]
		mat3[3][1] = vec1[1]
		mat3[3][2] = vec1[2]
		mat4 = Mat4(mat3[0][0], mat3[0][1], mat3[0][2], mat3[0][3], \
			mat3[1][0], mat3[1][1], mat3[1][2], mat3[1][3], \
			mat3[2][0], mat3[2][1], mat3[2][2], mat3[2][3], \
			mat3[3][0], mat3[3][1], mat3[3][2], mat3[3][3])
		self.jetLeftAileronNP.setMat(mat4)

		# RIGHT AILERON


		vec1 = self.jetCenterMatrix.xformPoint(Point3(-point+point*cos(radians(self.rightAileronAngle)), 0, point*sin(radians(self.rightAileronAngle))))
		mat1 = numpy.array(self.jetCenterMatrix)
		mat2 = self.rightAileronRotationMatrix
		mat2 = numpy.array(mat2)
		mat3 = mat1.dot(mat2)
		mat3[3][0] = vec1[0]
		mat3[3][1] = vec1[1]
		mat3[3][2] = vec1[2]
		mat4 = Mat4(mat3[0][0], mat3[0][1], mat3[0][2], mat3[0][3], \
			mat3[1][0], mat3[1][1], mat3[1][2], mat3[1][3], \
			mat3[2][0], mat3[2][1], mat3[2][2], mat3[2][3], \
			mat3[3][0], mat3[3][1], mat3[3][2], mat3[3][3])
		self.jetRightAileronNP.setMat(mat4)


	def update(self):
		"""Funkcja aktualizująca stan samolotu"""
		
		self.keysUpdate() # Aktualizacja stanu klawiszy i reakcji im przypisanych;

		self.jetNP.detachNode()
		self.jetElevatorNP.detachNode()
		self.jetRudderNP.detachNode()
		self.jetLeftAileronNP.detachNode()
		self.jetRightAileronNP.detachNode()

		self.speedText.clearText()
		self.thrustText.clearText()
		self.attitudeText.clearText()
		self.angleText.clearText()



		self.vehicle.setSteeringValue(-self.rudderAngle, 0)
		if self.thrust <= 0:
			self.vehicle.setBrake(100, 0)
			self.vehicle.setBrake(100, 1)
			self.vehicle.setBrake(100, 2)
		elif self.thrust > 0:
			self.vehicle.setBrake(0, 0)
			self.vehicle.setBrake(0, 1)
			self.vehicle.setBrake(0, 2)

		self.jetVelocityVector = self.jetNP.node().getLinearVelocity()
		self.jetVelocityVectorNorm = self.jetVelocityVector.normalized()
		self.jetVelocityValue = self.jetVelocityVector.length()

		self.jetCenterMatrixOld = self.jetCenterMatrix
		self.jetCenterPosOld = self.jetCenterMatrixOld.getRow3(3)
		self.jetCenterMatrix = self.jetNP.getNetTransform().getMat() # Zapisanie macierzy zawierającej położenie w przestrzeni i obroty punktu węzłowego self.jetNP;
		
		self.jetFrontVector = self.jetCenterMatrix.xformVec(self.jetFrontNP.getPos()) # Wektor ze zwrotem biegnącym od środka samolotu do początku jego dzioba;
		self.jetFrontVectorNorm = Vec3(self.jetFrontVector.normalized()) # Powyższy wektor unormowany; 
		self.jetFrontPoint = self.jetCenterMatrix.xformPoint(self.jetFrontNP.getPos())

		self.jetCenterVectorAbs = Vec3(self.jetCenterMatrix.getRow3(3))
		self.jetCenterVectorNorm = Vec3(self.jetCenterVectorAbs.normalized())
		self.jetCenterPosDiff = Vec3(self.jetCenterVectorAbs - self.jetCenterPosOld)
		self.jetCenterPosDiffNorm = Vec3(self.jetCenterPosDiff.normalized())


		self.jetLeftWingEdgeVector = self.jetCenterMatrix.xformVec(self.jetLeftWingEdgeNP.getPos()) 	#Pozycja krawędzi lewego skrzydła względem środka samolotu w absolutnym układzie współrzędnych.
		self.jetLeftWingEdgeVectorNorm = Vec3(self.jetLeftWingEdgeVector.normalized())

		self.jetRightWingEdgeVector = self.jetCenterMatrix.xformVec(self.jetRightWingEdgeNP.getPos()) 	#Pozycja krawędzi lewego skrzydła względem środka samolotu w absolutnym układzie współrzędnych.
		self.jetRightWingEdgeVectorNorm = Vec3(self.jetRightWingEdgeVector.normalized())

		self.jetUpVector = self.jetCenterMatrix.xformVec(Vec3(0,0,1)) 
		self.jetUpVectorNorm = Vec3(self.jetUpVector.normalized())

		self.jetRearVector = self.jetCenterMatrix.xformVec(self.jetRearNP.getPos())
		self.jetRearVectorNorm = Vec3(self.jetRearVector.normalized())
		self.jetRearPoint = self.jetCenterMatrix.xformPoint(self.jetRearNP.getPos())

		self.jetElevatorsPoint = self.jetCenterMatrix.xformVec(Point3(-3,0,0))
		self.jetRudderPoint = self.jetCenterMatrix.xformVec(Point3(-6,0,0))
		self.jetStabilizerPoint = self.jetCenterMatrix.xformVec(Point3(-2,0,0))
		self.jetWingsPoint = self.jetCenterMatrix.xformVec(Point3(2,0,0))

		self.rudderAngleVector = self.jetCenterMatrix.xformVec(Vec3(-0.45399, 0, -0.891007))
		self.rudderAngleVectorNorm = Vec3(self.rudderAngleVector.normalized())

		self.rudderFrontVector = self.jetCenterMatrix.xformVec(Vec3(0.891007, 0, 0.45399))
		self.rudderFrontVectorNorm = Vec3(self.rudderFrontVector.normalized())

		# Kąt natarcia w płaszczyźnie pionowej
		self.angleOfAttackVertical = round(self.jetVelocityVectorNorm.signedAngleDeg(\
			self.jetFrontVectorNorm, self.jetRightWingEdgeVectorNorm),2)
		# Kąt natarcia w płaszczyźnie poziomej
		self.angleOAttackHorizontal = round(self.jetVelocityVectorNorm.signedAngleDeg(\
			self.jetFrontVectorNorm, -self.jetUpVectorNorm), 2)

		### ELEVATOR

		# Zdefiniowanie macierzy obrotu:
		self.elevatorsRotationMatrix = Mat4()
		# Zapisanie w macierzy zadanego obrotu powierzchni sterownej:
		self.elevatorsRotationMatrix.setRotateMatNormaxis( \
			self.elevatorsAngle, self.jetRightWingEdgeVectorNorm)
		# Utworzenie wektora skierowanego pod w/w kątem:
		self.elevatorsAbsoluteVector = \
			self.elevatorsRotationMatrix.xformVec(self.jetFrontVectorNorm)
		# Normalizacja w/w wektora:
		self.elevatorsAbsoluteVectorNorm = \
			self.elevatorsAbsoluteVector.normalized()
		# Obliczenie kąta pomiędzy w/w wektorem a wektorem prędkości:
		self.elevatorsAbsoluteAngle = round(self.jetVelocityVectorNorm.signedAngleDeg( \
			self.elevatorsAbsoluteVectorNorm, self.jetRightWingEdgeVectorNorm),2)
		# Obliczenie skierowanego w górę wektora prostopadłego do powierzchni sterownej:
		self.elevatorsAbsoluteVectorOrto = \
			self.elevatorsRotationMatrix.xformVec(self.jetUpVectorNorm)
		# Normalizacja w/w wektora:
		self.elevatorsAbsoluteVectorOrtoNorm = \
			self.elevatorsAbsoluteVectorOrto.normalized()
		# Obliczanie współczynnika siły nośnej:
		coeff = self.liftCoeffCalculate(self.elevatorsAbsoluteAngle)
		# Obliczanie siły nośnej:
		self.elevatorsForceValue = 0.5 * self.environment.airDensity \
			* self.elevatorsArea * (self.jetVelocityValue)**2 * coeff
		# Pomnożenie siły nośnej przez wektor jednostkowy o zwrocie do góry płaszczyzny:
		self.elevatorsLiftVector = \
			self.elevatorsAbsoluteVectorOrtoNorm * self.elevatorsForceValue 
		# Przyłożenie wektora siły nośnej płaszczyzny sterownej do modelu samolotu:
		self.jetNP.node().applyForce(self.elevatorsLiftVector, self.jetElevatorsPoint)

		coeff = self.dragCoeffCalculate(self.elevatorsAbsoluteAngle)
		self.elevatorsDragForceValue = 0.5*self.environment.airDensity*self.elevatorsArea*(self.jetVelocityValue)**2*coeff
		self.elevatorsDragVector = -self.jetVelocityVectorNorm * self.elevatorsDragForceValue
		self.jetNP.node().applyForce(self.elevatorsDragVector, self.jetElevatorsPoint)


		### WINGS

		# Obliczanie współczynnika siły nośnej:
		coeff = self.liftCoeffCalculate(self.angleOfAttackVertical)
		# Obliczanie siły nośnej:
		self.liftForceValue = 0.5 * self.environment.airDensity \
		* self.wingsStaticArea * (self.jetVelocityValue)**2 * coeff
		# Pomnożenie siły nośnej przez wektor jednostkowy o zwrocie do góry samolotu:
		self.jetLiftVector = self.jetUpVectorNorm * self.liftForceValue 
		# Przyłożenie wektora siły nośnej skrzydeł do modelu samolotu:
		self.jetNP.node().applyForce(self.jetLiftVector, self.jetWingsPoint)

		# Obliczanie współczynnika siły oporu:
		coeff = self.dragCoeffCalculate(self.angleOfAttackVertical)
		# Obliczanie siły oporu:
		self.wingsDragForceValue = 0.5 * self.environment.airDensity \
		* self.wingsStaticArea * (self.jetVelocityValue)**2 * coeff
		# Pomnożenie siły oporu przez wektor jednostkowy 
		# o zwrocie przeciwnym do prędkości samolotu:
		self.jetDragVector = -self.jetVelocityVectorNorm * self.wingsDragForceValue
		# Przyłożenie wektora siły oporu skrzydeł do modelu samolotu:
		self.jetNP.node().applyForce(self.jetDragVector, self.jetWingsPoint)

		### LEFT AILERON

		self.leftAileronRotationMatrix = Mat4()
		self.leftAileronRotationMatrix.setRotateMatNormaxis(self.leftAileronAngle, self.jetRightWingEdgeVectorNorm)
		self.leftAileronAbsoluteVector = self.leftAileronRotationMatrix.xformVec(self.jetFrontVectorNorm)
		self.leftAileronAbsoluteVectorNorm = self.leftAileronAbsoluteVector.normalized()
		self.leftAileronAbsoluteAngle = round(self.jetVelocityVectorNorm.signedAngleDeg(self.leftAileronAbsoluteVectorNorm, self.jetRightWingEdgeVectorNorm),2)
		self.leftAileronAbsoluteVectorOrto = self.leftAileronRotationMatrix.xformVec(self.jetUpVectorNorm)
		self.leftAileronAbsoluteVectorOrtoNorm = self.leftAileronAbsoluteVectorOrto.normalized()

		coeff = self.liftCoeffCalculate(self.leftAileronAbsoluteAngle)
		self.leftAileronForceValue = 0.5*self.environment.airDensity*self.leftAileronArea*(self.jetVelocityValue)**2*coeff
		self.leftAileronLiftVector = self.leftAileronAbsoluteVectorOrtoNorm *self.leftAileronForceValue 
		self.jetNP.node().applyForce(self.leftAileronLiftVector, self.jetLeftWingEdgeVector)

		coeff = self.dragCoeffCalculate(self.leftAileronAbsoluteAngle)
		self.leftAileronDragForceValue = 0.5*self.environment.airDensity*self.leftAileronArea*(self.jetVelocityValue)**2*coeff
		self.leftAileronDragVector = -self.jetVelocityVectorNorm * self.leftAileronDragForceValue
		self.jetNP.node().applyForce(self.leftAileronDragVector, self.jetLeftWingEdgeVector)

		### RIGHT AILERON

		self.rightAileronRotationMatrix = Mat4()
		self.rightAileronRotationMatrix.setRotateMatNormaxis(self.rightAileronAngle, self.jetRightWingEdgeVectorNorm)
		self.rightAileronAbsoluteVector = self.rightAileronRotationMatrix.xformVec(self.jetFrontVectorNorm)
		self.rightAileronAbsoluteVectorNorm = self.rightAileronAbsoluteVector.normalized()
		self.rightAileronAbsoluteAngle = round(self.jetVelocityVectorNorm.signedAngleDeg(self.rightAileronAbsoluteVectorNorm, self.jetRightWingEdgeVectorNorm),2)
		self.rightAileronAbsoluteVectorOrto = self.rightAileronRotationMatrix.xformVec(self.jetUpVectorNorm)
		self.rightAileronAbsoluteVectorOrtoNorm = self.rightAileronAbsoluteVectorOrto.normalized()

		coeff = self.liftCoeffCalculate(self.rightAileronAbsoluteAngle)
		self.rightAileronForceValue = 0.5*self.environment.airDensity*self.rightAileronArea*(self.jetVelocityValue)**2*coeff
		self.rightAileronLiftVector = self.rightAileronAbsoluteVectorOrtoNorm * self.rightAileronForceValue 
		self.jetNP.node().applyForce(self.rightAileronLiftVector, self.jetRightWingEdgeVector)

		coeff = self.dragCoeffCalculate(self.rightAileronAbsoluteAngle)
		self.rightAileronDragForceValue = 0.5*self.environment.airDensity*self.leftAileronArea*(self.jetVelocityValue)**2*coeff
		self.rightAileronDragVector = -self.jetVelocityVectorNorm * self.rightAileronDragForceValue
		self.jetNP.node().applyForce(self.rightAileronDragVector, self.jetRightWingEdgeVector)


		### RUDDER

		self.rudderRotationMatrix = Mat4()
		self.rudderRotationMatrix.setRotateMatNormaxis(self.rudderAngle, self.jetUpVectorNorm)
		self.rudderAbsoluteVector = self.rudderRotationMatrix.xformVec(self.jetFrontVectorNorm)
		self.rudderAbsoluteVectorNorm = self.rudderAbsoluteVector.normalized()

		self.rudderAbsoluteAngle = round(self.jetVelocityVectorNorm.signedAngleDeg(self.rudderAbsoluteVectorNorm, self.jetUpVectorNorm),2)
		self.rudderAbsoluteVectorOrto = self.rudderRotationMatrix.xformVec(self.jetLeftWingEdgeVectorNorm)
		self.rudderAbsoluteVectorOrtoNorm = self.rudderAbsoluteVectorOrto.normalized()

		coeff = self.rudderCoeffCalculate(self.rudderAbsoluteAngle)
		self.rudderForceValue = 0.5*self.environment.airDensity*self.rudderArea*(self.jetVelocityValue)**2*coeff
		self.rudderLiftVector = self.rudderAbsoluteVectorOrtoNorm * self.rudderForceValue 
		self.jetNP.node().applyForce(self.rudderLiftVector, self.jetRudderPoint)

		coeff = self.dragCoeffCalculate(self.rudderAbsoluteAngle)
		self.rudderDragForceValue = 0.5*self.environment.airDensity*self.rudderArea*(self.jetVelocityValue)**2*coeff
		self.rudderDragVector = -self.jetVelocityVectorNorm * self.rudderDragForceValue
		self.jetNP.node().applyForce(self.rudderDragVector, self.jetRudderPoint)

		### VERTICAL STABILIZER
		
		coeff = self.rudderCoeffCalculate(self.angleOAttackHorizontal)
		self.verticalStabilizerForceValue = 0.5*self.environment.airDensity*self.verticalStabilizerArea*(self.jetVelocityValue)**2*coeff
		self.verticalStabilizerLiftVector = self.jetRightWingEdgeVectorNorm * self.verticalStabilizerForceValue 
		self.jetNP.node().applyForce(self.verticalStabilizerLiftVector, self.jetStabilizerPoint)

		coeff = self.dragCoeffCalculate(self.angleOAttackHorizontal)
		self.verticalStabilizerDragForceValue = 0.5*self.environment.airDensity*self.verticalStabilizerArea*(self.jetVelocityValue)**2*coeff
		self.verticalStabilizerDragVector = -self.jetVelocityVectorNorm * self.verticalStabilizerDragForceValue
		self.jetNP.node().applyForce(self.verticalStabilizerDragVector, self.jetStabilizerPoint)

		### THRUST

		self.jetThrustVector = self.jetFrontVectorNorm * self.thrust 		# Wektor o wartości siły ciągu silnika self.thrust o kierunku zgodnym z wekorem self.jetFrontVector 
		self.jetNP.node().applyCentralForce(self.jetThrustVector)


		self.flapsSettings()




		self.jetNP = self.environment.render.attachNewNode(self.jetNP.node())
		self.jetElevatorNP = self.environment.render.attachNewNode(self.jetElevatorNP.node())
		self.jetRudderNP = self.environment.render.attachNewNode(self.jetRudderNP.node())
		self.jetLeftAileronNP = self.environment.render.attachNewNode(self.jetLeftAileronNP.node())
		self.jetRightAileronNP = self.environment.render.attachNewNode(self.jetRightAileronNP.node())

		self.lookAtPos = self.jetNP.getPos()
		self.camPos = self.jetNP.getPos() + Point3(0, 0, 30)
		# self.camPos_1 = self.jetCenterMatrix.xformPoint(Point3(-5, -45, 0))
		self.camPos_1 = self.jetCenterMatrix.xformPoint(Point3(0, -45, 0))
		self.camPos_2 = self.jetCenterMatrix.xformPoint(Point3(-40-0.1*self.jetVelocityValue, 0, 5 - 0.2 * self.elevatorsAngle))
		self.camPos_3 = self.jetCenterMatrix.xformPoint(Point3(40+0.1*self.jetVelocityValue, 0, 5 - 0.2 * self.elevatorsAngle))
		self.camPos_4 = self.jetNP.getPos() + Point3(100, 50, -self.jetNP.getPos()[2] + 50)
		self.camPos_5 = self.jetCenterMatrix.xformPoint(Point3(-40-0.1*self.jetVelocityValue, 0, 15 - 0.2 * self.elevatorsAngle))

		# self.lines[1].update(self.verticalStabilizerLiftVector.normalized(), self.jetRearPoint)		
		# self.lines[2].update(self.rudderAbsoluteVectorNorm, self.jetNP.getPos())
		# self.lines[3].update(self.jetVelocityVectorNorm*1.5, self.jetNP.getPos())

		text = "Predkosc: "+str(int(self.jetVelocityValue))+" m/s"
		self.speedText.setText(text)

		text = "Sila ciagu silnika: "+str(int(self.thrust))+" N"
		self.thrustText.setText(text)

		text = "Wysokosc: "+str(int(self.jetNP.getZ()-2.83))+" m"
		self.attitudeText.setText(text)

		if self.jetVelocityValue > 0.5:
			text = "Kat natarcia (wzgledem wektora predkosci): "+str(int(self.angleOfAttackVertical))+" "+chr(176)
		else:
			text = "Kat natarcia (wzgledem wektora predkosci): 0 "+chr(176)
		self.angleText.setText(text)



class DrawVector():
	def __init__(self, environment, jet, vectorLength, vectorColor = Vec4(0,1,1,1)):

		self.environment = environment
		self.jet = jet
		self.vectorLength = vectorLength

		self.line1 = LineSegs()
		self.line1.setColor(vectorColor)
		self.node1 = self.line1.create()
		self.np = self.environment.render.attachNewNode(self.node1)
		self.np.reparentTo(self.environment.render)		
		self.np.detachNode()

	def update(self, vectorNorm, endPoint):
		self.np.detachNode()
		self.line1.moveTo(vectorNorm * self.vectorLength + endPoint) 
		self.line1.drawTo(endPoint)
		self.node1 = self.line1.create()
		self.np = self.environment.render.attachNewNode(self.node1)



class LocalCoordinateSystem():
	"""EN: Class which defines local coordinate system of jet.
	   PL: Klasa definiująca lokalny układ odniesienia odrzutowca."""

	def __init__(self, environment, jet, axisLength):
		self.environment = environment
		self.jet = jet
		self.axisLength = axisLength
		self.visible = False
		self.ortho = False

		self.posX = Point3(self.axisLength, 0, 0)
		self.posY = Point3(0, self.axisLength, 0)
		self.posZ = Point3(0, 0, self.axisLength)

		self.pivotNode_X = self.environment.render.attachNewNode("environ-pivot")
		self.pivotNode_X.setPos(self.posX)
		self.pivotNode_X.setHpr(0, 0, 0)

		self.pivotNode_Y = self.environment.render.attachNewNode("environ-pivot")
		self.pivotNode_Y.setPos(self.posY)
		self.pivotNode_Y.setHpr(0, 0, 0)

		self.pivotNode_Z = self.environment.render.attachNewNode("environ-pivot")
		self.pivotNode_Z.setPos(self.posZ)
		self.pivotNode_Z.setHpr(0, 0, 0)

		self.pivotNode_XYZ = self.environment.render.attachNewNode("environ-pivot")
		self.pivotNode_XYZ.setPos(0, 0, 0)
		self.pivotNode_XYZ.setHpr(0, 0, 0)

		self.lineX = LineSegs()
		self.lineX.setColor( Vec4(0,0,1,1) ) #blue
		self.nodeX = self.lineX.create()
		self.npX = self.environment.render.attachNewNode(self.nodeX)
		self.npX.reparentTo(self.environment.render)

		self.lineY = LineSegs()
		self.lineY.setColor( Vec4(1,1,0,1) ) #yellow
		self.nodeY = self.lineY.create()
		self.npY = self.environment.render.attachNewNode(self.nodeY)
		self.npY.reparentTo(self.environment.render)

		self.lineZ = LineSegs()
		self.lineZ.setColor( Vec4(1,0,0,1) ) #red
		self.nodeZ = self.lineZ.create()
		self.npZ = self.environment.render.attachNewNode(self.nodeZ)
		self.npZ.reparentTo(self.environment.render)

		self.lineXYZ = LineSegs()
		self.lineXYZ.setColor( Vec4(0,1,1,1) )
		self.nodeXYZ = self.lineXYZ.create()
		self.npXYZ = self.environment.render.attachNewNode(self.nodeXYZ)
		self.npXYZ.reparentTo(self.environment.render)		

	def update(self):

		if self.ortho:
			self.axisXend = self.jet.jetNP.getPos() + self.posX
			self.axisYend = self.jet.jetNP.getPos() + self.posY
			self.axisZend = self.jet.jetNP.getPos() + self.posZ
		else:
			self.axisXend = self.jet.jetCenterMatrix.xform(LVector4f(self.pivotNode_X.getPos())).getXyz()
			self.axisYend = self.jet.jetCenterMatrix.xform(LVector4f(self.pivotNode_Y.getPos())).getXyz()
			self.axisZend = self.jet.jetCenterMatrix.xform(LVector4f(self.pivotNode_Z.getPos())).getXyz()

		# BLUE
		self.npX.detachNode()
		self.lineX.moveTo(self.axisXend)
		self.lineX.drawTo(self.jet.jetCenterMatrix.getRow3(3))
		self.nodeX = self.lineX.create()
		self.npX = self.environment.render.attachNewNode(self.nodeX)

		# YELLOW
		self.npY.detachNode()
		self.lineY.moveTo(self.axisYend)
		self.lineY.drawTo(self.jet.jetCenterMatrix.getRow3(3))
		self.nodeY = self.lineY.create()
		self.npY = self.environment.render.attachNewNode(self.nodeY)

		# RED
		self.npZ.detachNode()
		self.lineZ.moveTo(self.axisZend)
		self.lineZ.drawTo(self.jet.jetCenterMatrix.getRow3(3))
		self.nodeZ = self.lineZ.create()
		self.npZ = self.environment.render.attachNewNode(self.nodeZ)

		# WEKTOR SIŁ WYPADKOWYCH
		self.npXYZ.detachNode()
		self.lineXYZ.moveTo(self.jet.jetNP.node().getTotalForce().normalized() * 10 + self.jet.jetCenterMatrix.getRow3(3)) 
		self.lineXYZ.drawTo(self.jet.jetCenterMatrix.getRow3(3))
		self.nodeXYZ = self.lineXYZ.create()
		self.npXYZ = self.environment.render.attachNewNode(self.nodeXYZ)

		if self.visible:
			pass
		else:
			self.npX.detachNode()
			self.npY.detachNode()
			self.npZ.detachNode()
			self.npXYZ.detachNode()

	def switchCS(self):
		if self.ortho:
			self.ortho = False
		else:
			self.ortho = True

	def toggleCS(self):
		if self.visible:
			self.visible = False
		else:
			self.visible = True



class Light():
	"""EN: Class which defines lightning.
	   PL: Klasa definiująca oświetlenie."""

	def __init__(self, environment):

		# pass
		self.environment = environment
		self.ambientLight = AmbientLight("ambientLight")
		self.ambientLight.setColor(Vec4(.25, .25, .25, 1))

		self.directionalLight = DirectionalLight("directionalLight")
		self.directionalLight.setColor(Vec4(1, 1, 1, 1))
		self.directionalLight.setSpecularColor(Vec4(1, 1, 1, 1))
		self.np = self.environment.render.attachNewNode(self.directionalLight)
		self.np.set_pos(self.environment.jet.camPos)
		self.np.look_at(self.environment.jet.lookAtPos)

		self.directionalLight.get_lens().set_near_far(1, 50)
		self.directionalLight.get_lens().set_film_size(100, 100)
		# self.directionalLight.show_frustum()
		self.directionalLight.set_shadow_caster(True, 4096, 4096)

		self.environment.render.setLight(self.environment.render.attachNewNode(self.ambientLight))
		self.environment.render.setLight(self.np)	
		self.environment.render.setShaderAuto()

	def update(self):
		self.np.set_pos(self.environment.jet.camPos)
		self.np.look_at(self.environment.jet.lookAtPos)
		self.np.set_hpr(10,-135,0)



class Debug():
	"""EN: Class for debuging purpouses.
	   PL: Klasa do debugowania aplikacji."""

	def __init__(self, environment):
		self.environment = environment
		self.debugNode = BulletDebugNode('Debug')
		self.debugNode.showWireframe(True)
		self.debugNode.showConstraints(True)
		self.debugNode.showBoundingBoxes(False)
		self.debugNode.showNormals(False)
		self.debugNP = self.environment.render.attachNewNode(self.debugNode)
		self.environment.world.setDebugNode(self.debugNP.node())
		# self.debugNP.show()

	def toggleDebug(self):
		if self.debugNP.isHidden():
			self.debugNP.show()
		else:
			self.debugNP.hide()



env = App()
env.run()
