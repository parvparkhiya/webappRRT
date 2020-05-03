from django.shortcuts import render
from django.http import HttpResponse

from .models import Greeting
from RRTApp.RRT import runRRT

# Create your views here.
def index(request):
    # return HttpResponse('Hello from Python!')
    runRRT()
    data = {'appName':"RRT", 'author':"Parv", 'imageName':"rrt.png"}
    return render(request, "home.html", data)


def db(request):

    greeting = Greeting()
    greeting.save()

    greetings = Greeting.objects.all()

    return render(request, "db.html", {"greetings": greetings})
