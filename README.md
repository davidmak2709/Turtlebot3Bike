# Turtlebot3 - Bike

Upute za postavljanje Turlebot3 robota u Bike konfiguraciji. Službeni e-manual na kojeg se ove upute često referenciraju nalaze se [OVDJE -Službene Turtlebot3 upute](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview).

## 1. Slaganje

Robot složite slijedeći Turtlebot3 upute koje se nalaze u kutiji robota te pokušavajući emulirati bike dizajn koji je prikazan na slici. Pritom pazite da ostavite dovoljno mjesta za sve potrebne komponente koje spajamo od dna prema vrhu slijedećim redosljedom:
* Baterija
* OpenCR
* Raspberry Pi
* Lidar - laserski skrener

![Slike bike dizajn](http://emanual.robotis.com/assets/images/platform/turtlebot3/friends/friends_bike.png)

Na ovom linku  možete pronači CAD model [Turtlebot3 Bike](https://cad.onshape.com/documents/e0db6e7fac208692fd867efa/w/c9f5e7c3e86bd0089cc1fc00/e/eb045c245b85ed8c9a0ad494). 

## 2. Postavljanje motora i OpenCR

Prije programiranja OpenCR pločice koja služi kao kontroler za Dynamixel motore potrebno je postaviti ID svih motora te promijeniti modalitet rada prednjeg motora. 

ID motora mora odgovarati konfiguraciji na slici, koje postavljamo korištenjem programa iz Arduino IDE `File -> Examples -> OpenCR -> DynamixelWorkbench -> d_ID_Change` .


![ID motora](https://raw.githubusercontent.com/davidmak2709/Turtlebot3Bike/images/ids.png?token=AZK12jzNIelTguBWG_-IZBmnK-SxAfesks5bLNV4wA%3D%3D)

Motoru s  ID-jem 3 potrebno je promjeniti način rada iz wheel mode u joint mode isto tako korištenjem OpenCR i defaultnog programa u Arduino IDE `File -> Examples -> OpenCR -> DynamixelWorkbench -> f_Mode_Change` .

Po završteku procesa postavljanja motora na OpenCR pločicu uploadamo program koji se nalazi u bike direktoriju ovog Git repozitorija.

## 3. Raspberry Pi

Raspberry Pi postavite slijedeći upute na službenom e-Manualu slijedeći poglavlje [6.2.1. Raspberry Pi 3 Setup](http://emanual.robotis.com/docs/en/platform/turtlebot3/raspberry_pi_3_setup/#raspberry-pi-3-setup) te odaberite Ubuntu Mate.

## 4. PC

Na udaljenom račuanlo koje će upravljati robotom potrebno je instalirati sve potrebne pakete slijedeći poglavlje [6.1. PC Setup](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc-setup)

## 5. KRENI!!

Prije pokretanja bilo koje od funkcionalnosti (Teleop, SLAM, Navigacija) potrebno je pokrenuti sve naredbe u poglavlju [7. Bringup](http://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup).

Za korištenje robota i njegovih funkcionalnosti udaljeno upravljanje (Teleop), SLAM-a i Navigacije se izvršava prateći poglavlja na službenom eManual [8. Basic Operation](http://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation), [9. SLAM](http://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam) i [10. Navigation](emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation).

![Rezultat](https://raw.githubusercontent.com/davidmak2709/Turtlebot3Bike/images/result.jpg?token=AZK12u7CUszmOCo-tp1N7u1IOxgMelsEks5bLNWlwA%3D%3D)

## 6.Dodatak na projekt

//TODO git repo za PUS


