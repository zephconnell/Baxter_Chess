#A program to detect human features using
#Haar feature based cascade classifiers
#The available human Haar feature classifiers are
#fullybody.xml
#face.xml
#upper_body.xml
#right_ear.xml
#eye.xml
#lower_body.xml
#nose.xml
#mouth.xml

from SimpleCV import *
from Tkinter import * #import Tkinter package for Python2

#Basic Window Class
class Application(Frame):
    """Build the basic window frame template"""
    def __init__(self,master=None):
        Frame.__init__(self,master) #Create Window in Python2 version of Tkinter
        self.c = Camera()   #Create a USB connected Camera object
        self.ImageDisp = 0
        self.grid()
        self.create_widgets()   #Call function to create buttons.      

    #A function to create graphical user interface of the application
    def create_widgets(self):
        #Create a button with the name "Picture"
        #The function OnPicture will be called when the button is pressed
        self.buttonPicture = Button(self,text="Take Picture",font=("Helvetica",32),command = self.OnPicture)
        #Place the button at first row and first column
        self.buttonPicture.grid(row=0,column=0,sticky=W)

        #Create a button with the name "Detect Faces"
        #The function OnDetectFace will be called when the button is pressed
        self.buttonFace = Button(self,text="Detect Faces",font=("Helvetica",32),command = self.OnDetectFace)
        #Place the button at second row and first column
        self.buttonFace.grid(row=1,column=0,sticky=W)


        #Create a button with the name "Detect Nose"
        #The function OnDetectNose will be called when the button is pressed
        self.buttonNose = Button(self,text="Detect Nose",font=("Helvetica",32),command = self.OnDetectNose)
        #Place the button at second row and Second column
        self.buttonNose.grid(row=1,column=1,sticky=W)

        #Create a button with the name "Detect Mouth"
        #The function OnDetectMouth will be called when the button is pressed
        self.buttonMouth = Button(self,text="Detect Mouth",font=("Helvetica",32),command = self.OnDetectMouth)
        #Place the button at third row and first column
        self.buttonMouth.grid(row=2,column=0,sticky=W)

        #Create a button with the name "Detect Upper Body"
        #The function OnDetectUpperBody will be called when the button is pressed
        self.buttonUpperBody = Button(self,text="Detect Upper Body",font=("Helvetica",32),command = self.OnDetectUpperBody)
        #Place the button at third row and first column
        self.buttonUpperBody.grid(row=2,column=1,sticky=W)

        #Create a button with the name "Detect Upper Body"
        #The function OnDetectUpperBody will be called when the button is pressed
        self.buttonExit = Button(self,text="Exit",font=("Helvetica",32),command = self.OnExit)
        #Place the button at fourth row and first column
        self.buttonExit.grid(row=4,column=1,sticky=W)


        
        
    def OnPicture(self):
        #Take a picture and display
        self.img = self.c.getImage()
        self.ImageDisp = self.img.show()

    def OnDetectFace(self):
        #Detect faces in the picture using the function findHaarFeatures
        #face.xml indicates to detect face features
        #min_neighbors control the number of false positive recognitions
        #Lower value of min_neighbors will increase the number of false positives
        self.faces = self.img.findHaarFeatures('face.xml',min_neighbors=5)
        #Draw squares around the detected faces with red color
        self.faces.draw(color=(255,0,0),width=8)
        #Draw the detected faces on the original image
        self.ImageDisp = self.img.show()
        self.x_faces = self.faces.x()
        self.y_faces = self.faces.y()
        self.face_widths = self.faces.width()


    def OnDetectNose(self):
        #Detect Nose in the picture
        noses = self.img.findHaarFeatures('nose.xml',min_neighbors=5)
        x_noses = noses.x()
        y_noses = noses.y()
        nose_areas = noses.area()
        nose_widths = noses.width()
        nose_lengths = noses.length()
##        noses.draw(color=(0,255,0),width=8)
##        self.ImageDisp = self.img.show()

        face_count = len(self.x_faces)
        nose_count = len(x_noses)
        if(nose_count>0):
            for i in range(0,face_count):
                x_face = self.x_faces[i]
                y_face = self.y_faces[i]
                length = self.face_widths[i]

                max_idx = 0
                max_area = 0
                found = False
                
                for j in range(0,nose_count):
                    x_nose = x_noses[j]
                    y_nose = y_noses[j]
                    nose_area = nose_areas[j]

                    if(x_nose<x_face-length/2 or x_nose>x_face+length/2 or y_nose<y_face-length/2 or y_nose>y_face+length/2):
                        continue
                    elif(nose_area>max_area):
                        max_idx = j
                        max_area = nose_area
                        found = True

                if(found):
                    facelayer = DrawingLayer((self.img.width, self.img.height))
                    facebox_dim = (nose_widths[max_idx],nose_lengths[max_idx])
                    center_point = (x_noses[max_idx], y_noses[max_idx])
                    facebox = facelayer.centeredRectangle(center_point, facebox_dim,color=(0,0,255),width=8)
                    self.img.addDrawingLayer(facelayer)
                    self.img.applyLayers()
                    self.img.show()
                 

    def OnDetectMouth(self):
        #Detect mouth in the picture
        self.mouth = self.img.findHaarFeatures('mouth.xml',min_neighbors=5)
##        self.mouth.draw(color=(0,0,255),width=8)
##        self.ImageDisp = self.img.show()

        x_mouths = self.mouth.x()
        y_mouths = self.mouth.y()
        mouth_areas = self.mouth.area()
        mouth_widths = self.mouth.width()
        mouth_lengths = self.mouth.length()
        
        face_count = len(self.x_faces)
        mouth_count = len(x_mouths)

        if(mouth_count>0):
            for i in range(0,face_count):   
                x_face = self.x_faces[i]
                y_face = self.y_faces[i]
                length = self.face_widths[i]

                max_idx = 0
                max_area = 0
                found = False;
                
                for j in range(0,mouth_count):
                    x_mouth = x_mouths[j]
                    y_mouth = y_mouths[j]
                    mouth_area = mouth_areas[j]


                    if(x_mouth<(x_face-length/2) or x_mouth>(x_face+length/2) or y_mouth<(y_face) or y_mouth>(y_face+length/2)):
                        continue
                    elif(mouth_area>max_area):
                         max_idx = j
                         max_area = mouth_area
                         found = True;

                if(found):
                    facelayer = DrawingLayer((self.img.width, self.img.height))
                    facebox_dim = (mouth_widths[max_idx],mouth_lengths[max_idx])
                    center_point = (x_mouths[max_idx], y_mouths[max_idx])
                    facebox = facelayer.centeredRectangle(center_point, facebox_dim,color=(0,255,0),width=8)
                    self.img.addDrawingLayer(facelayer)
                    self.img.applyLayers()
                    self.img.show()
        

    def OnDetectUpperBody(self):
        #Detect Upper Body in the picture
        self.UpperBody = self.img.findHaarFeatures('upper_body.xml',min_neighbors=5)
        self.UpperBody.draw(color=(255,255,0),width=8)
        self.ImageDisp = self.img.show()

    def OnExit(self):
        #Exit the program
        if(self.ImageDisp != 0):
            self.ImageDisp.quit()
        root.destroy()

#Create a Main Window Object        
root = Tk()
#Set the title of the Main Window
root.title('Human Feature Detection')
#Create a window with Width = 700, Height = 300, X Position = 50, Y Position = 50
root.geometry('700x300+50+50')
#Set the main window as the main maindow of the application
app = Application(master=root)
#Launch the application
app.mainloop()


