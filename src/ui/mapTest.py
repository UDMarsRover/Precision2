
import json


class Pin():
    def __init__(self, x, y, t):
        self.lat = x
        self.lon = y
        self.timeStamp = t
    def to_dict(self):
        return {"Lattitude" : self.lat, "Longitude":self.lon, "Time": self.timeStamp}

    def from_dict(cls, data):
        return cls(data["Lattitude"], data["Longitude"], data["Time"])
    
    def toString(self):
        return str(self.lat) + " " + str(self.lon) + " " + str(self.timeStamp)
    
pins = []

with open('pin_data.json', 'r') as f:
    pins_dicts = json.load(f)
    for data in pins_dicts:     
    #self.pins = [Pin.from_dict(Pin, data) for data in self.pins_dicts]
        pins.append(Pin.from_dict(Pin, data))
        
for i in pins:
    print(i.toString())
