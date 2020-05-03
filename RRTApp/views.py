from django.shortcuts import render
from django.http import HttpResponse

from .models import Greeting
from .forms import paramForm
from RRTApp.RRT import runRRT

def index(request):
    data = {'appName':"RRT", 'author':"Parv", 'imageName':"rrt.png"}

    goal = (-8.0, -8.0)
    start = (0.0, 0.0)
    stepLength = 0.5
    goalBias = 4

    if request.method == 'POST':
        form = paramForm(request.POST)
        if form.is_valid() and not form.cleaned_data['reset']:
            if form.cleaned_data['startRandom']:
                start = None
            else:
                start = (form.cleaned_data['startX'], form.cleaned_data['startY'])
            
            if form.cleaned_data['goalRandom']:
                goal = None
            else:
                goal = (form.cleaned_data['goalX'], form.cleaned_data['goalY'])
    
            # start = (form.cleaned_data['startX'], form.cleaned_data['startY'])
            # goal = (form.cleaned_data['goalX'], form.cleaned_data['goalY'])
            

            stepLength = form.cleaned_data['stepLength']
            goalBias = form.cleaned_data['goalBias']

    start, goal = runRRT(start, goal, stepLength, goalBias)

    data['goalX'] = round(goal[0], 2)
    data['goalY'] = round(goal[1], 2)
    data['startX'] = round(start[0], 2)
    data['startY'] = round(start[1], 2)
    data['stepLength'] = stepLength
    data['goalBias'] = goalBias

    return render(request, "home.html", data)


def db(request):

    greeting = Greeting()
    greeting.save()

    greetings = Greeting.objects.all()

    return render(request, "db.html", {"greetings": greetings})
