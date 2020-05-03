from django import forms
    
class paramForm(forms.Form):
    goalX = forms.FloatField(label="goalX")
    goalY = forms.FloatField(label="goalY")

    startX = forms.FloatField(label="startX")
    startY = forms.FloatField(label="startY")
    
    stepLength = forms.FloatField(label="stepLength", min_value = 0.1)
    goalBias = forms.IntegerField(label="goalBias", min_value = 2)

    startRandom = forms.BooleanField(label="startRandom", required=False)
    goalRandom = forms.BooleanField(label="goalRandom", required=False)

    reset = forms.BooleanField(label="reset", required=False)
