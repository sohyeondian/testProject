from pop import PopThread,Psd,PixelDisplay
import time

class Factory(PopThread):
    def __init__(self):
        super().__init__()
        self.px=PixelDisplay()
        self.psd=Psd()
        self.big_box = []
        self.small_box = []        
   
    def run(self): 
        for belt in range(5):
            self.px.setColor(belt, 4, [255,255,0])
            time.sleep(0.5)
            self.px.setColor(belt, 4, [0,0,0])
                
            val = self.psd.read()
            Product_size = self.psd.calcDist(val)
            big = len(self.big_box) % 8
            small = len(self.small_box) % 8

            if belt == 4:
                if Product_size >=50:
                    self.px.setColor(big, 0, [255,255,0])
                    self.big_box.append(1)
                    if big == 7:
                        for i in range(8):
                            self.px.setColor(i, 0, [0,0,0])
                                               
                elif Product_size <= 30:
                    self.px.setColor(small, 7, [255,255,0])
                    self.small_box.append(1)
                    if small == 7:
                        for j in range(8):
                            self.px.setColor(j, 7, [0,0,0])
    
        time.sleep(1)