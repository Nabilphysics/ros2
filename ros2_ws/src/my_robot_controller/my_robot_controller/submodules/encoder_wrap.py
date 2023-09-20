'''
This class takes wheel encoder ticks and takes care of wraping
Originally from https://github.com/jfstepha/differential-drive/blob/master/scripts/diff_tf.py#L192
Re-Written by Syed Razwanul Haque (Nabil)
https://www.github.com/Nabilphysics
'''
class EncoderWrap():
    def __init__(self, encoder_min, encoder_max):
        self.encoder_min = encoder_min
        self.encoder_max = encoder_max
        self.mult = 0 
        self.prev_encoder = 0
        self.curr_encoder = 0 
        self.encoder_low_wrap = ((self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min) 
        self.encoder_high_wrap = ((self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )


    def getEncTick(self, curr_value_from_mcu_enc):
            #global mult,prev_encoder,curr_enc,encoder_high_wrap, encoder_low_wrap
            encoder = curr_value_from_mcu_enc
            if (encoder < self.encoder_low_wrap and self.prev_encoder > self.encoder_high_wrap):
                self.mult = self.mult + 1
           
            if (encoder > self.encoder_high_wrap and self.prev_encoder < self.encoder_low_wrap):
                self.mult = self.mult - 1
                
            self.curr_encoder = 1.0 * (encoder + self.mult * (self.encoder_max - self.encoder_min)) 
            self.prev_encoder = encoder
            return self.curr_encoder