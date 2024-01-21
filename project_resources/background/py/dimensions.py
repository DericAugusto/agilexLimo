from tkinter import *
from PIL import Image, ImageTk

class Draw(Frame):
    "classe définissant la fenêtre principale du programme"
    def __init__(self):
        Frame.__init__(self)
        # mise en place du canevas :
        self.can = Canvas(self, width = 640, height = 640)
        self.can.grid(row = 0, column = 1)
        self.image=Image.open('plateformecoloriee.png')
        self.photo = ImageTk.PhotoImage(self.image)
        self.item = self.can.create_image(0, 0, anchor = NW, image=self.photo)
        print(self.image.size[0])
        self.pack()

if __name__ == '__main__':
    Draw().mainloop()