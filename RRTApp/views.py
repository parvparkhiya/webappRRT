from django.shortcuts import render
from django.http import HttpResponse

from .models import Greeting
from .forms import paramForm
from RRTApp.RRT import runRRT
from RRTApp.constrainedRRT import runConstrainedRRT 
import numpy as np

def index(request):
    data = {'appName':"Rapidly Exploring Random Trees (RRTs)", 'author':"Parv", 'imageName':"rrt.png", 'message':"Path Found!", 'constraint':"", 'startRandom':"", 'goalRandom':""}

    goal = [-8.0, 8.0, np.pi/2]
    start = [0.0, 0.0, 0.0]
    stepLength = 0.5
    goalBias = 4
    constrainedFlag = True
    randomStartFlag = False
    randomGoalFlag = False

    if request.method == 'POST':
        form = paramForm(request.POST)
        if form.is_valid():
            if not form.cleaned_data['reset']:
                if form.cleaned_data['startRandom']:
                    start = None
                    randomStartFlag = True
                else:
                    start[0] = (form.cleaned_data['startX'])
                    start[1] = (form.cleaned_data['startY'])
                    start[2] = np.deg2rad(form.cleaned_data['startT'])
                
                if form.cleaned_data['goalRandom']:
                    goal = None
                    randomGoalFlag = True
                else:
                    goal[0] = (form.cleaned_data['goalX'])
                    goal[1] = (form.cleaned_data['goalY'])
                    goal[2] = np.deg2rad(form.cleaned_data['goalT'])

                stepLength = form.cleaned_data['stepLength']
                goalBias = form.cleaned_data['goalBias']
                constrainedFlag = form.cleaned_data['constraint']
        else:
            data['message'] = "Invalid param. Setting Default Value."

    if constrainedFlag:
        start, goal, success = runConstrainedRRT(start, goal, stepLength, goalBias)
        data['goalT'] = round(np.rad2deg(goal[2]), 2)
        data['startT'] = round(np.rad2deg(start[2]), 2)
    else:
        start, goal, success = runRRT(start, goal, stepLength, goalBias)
        data['goalT'] = 0.0
        data['startT'] = 0.0

    if (constrainedFlag):
        data['constraint'] = "checked"

    if (randomStartFlag):
        data['startRandom'] = "checked"

    if (randomGoalFlag):
        data['goalRandom'] = "checked"

    data['goalX'] = round(goal[0], 2)
    data['goalY'] = round(goal[1], 2)
    data['startX'] = round(start[0], 2)
    data['startY'] = round(start[1], 2)
    data['stepLength'] = stepLength
    data['goalBias'] = goalBias

    if not success:
        data['message'] = "Error: Goal or Start is on the obstacle or Invalid Param"

    return render(request, "rrtWidget.html", data)


def db(request):

    greeting = Greeting()
    greeting.save()

    greetings = Greeting.objects.all()

    return render(request, "db.html", {"greetings": greetings})
