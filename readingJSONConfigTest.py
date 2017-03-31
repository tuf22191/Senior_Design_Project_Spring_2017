

#test7 calculates the Y and the X values of the velocity
#for motion detection


#now the rgb values are working
#help(flycapture2) , dirs(flycapture2) or obj inside instead, vars(obj)


import json

data = None
with open('tapeConfig.json') as jsonfile:

        data = json.load(jsonfile)
        jsonfile.close()

pixelToCentimeterTable = []
for i in range(11):
    pixelToCentimeterTable.append([ data[i]['avgx'] ,i*10])# pixel, then cm correspondence


# print data[0]['avgx']
# print data[1]['avgx']
# print data[2]['avgx']



print pixelToCentimeterTable
