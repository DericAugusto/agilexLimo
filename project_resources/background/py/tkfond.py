from tkinter import Tk, PhotoImage, Canvas
from PIL import Image, ImageTk

class Interface(Tk):
    def __init__(self, fond, image):
        super(Interface, self).__init__()

        self.fond = PhotoImage(file=fond)
        self.image = image
        self.w, self.h = self.fond.width(), self.fond.height()

        self.canvas = Canvas(self, width=self.w, height=self.h)
        self.canvas.pack()
        self.canvas.create_image((self.w//2, self.h//2), image=self.fond)
        self.canvas.create_image((0,0), image=self.image)

        self.mainloop()

img = (Image.open("robot.png"))
resized_image = img.resize(30,60)
robot= ImageTk.PhotoImage(resized_image)

Interface('plateformecoloriee.png',robot)