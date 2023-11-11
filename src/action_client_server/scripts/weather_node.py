#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import requests




def acquire_weather_data():
    

    # Set your OpenWeatherMap API key
    api_key = 'a87562299d69d234e17dc4541ee51b07'

    # Set the location for which you want to get the weather data
    print('''1. To know latitude and longitude go to maps application 
    2.Long press your loaction icon(Blue circle)"
    3. Youw ill find latitude(N) and longitude(E)''')
    latitude=float(input("Enter latitude (N) of your location : "))
    longitude=float(input('Enter longitude (E) of you location : '))
    # Send a GET request to the OpenWeatherMap API to get the current weather data for the location
    response = requests.get(f'http://api.openweathermap.org/data/2.5/weather?lat={latitude}&lon={longitude}&appid={api_key}')

    # Parse the JSON response to extract the weather condition
    weather_data = response.json()
    weather_condition = weather_data['weather'][0]['main']

    # Use the weather condition as an if-else condition to perform some operations
    if weather_condition == 'Clear':
        print(f"It's clear at the site.")
        # Perform some operations for clear weather condition
    elif weather_condition == 'Clouds':
        print(f"It's cloudy at the site.")
        # Perform some operations for cloudy weather condition
    elif weather_condition == 'Rain':
        print(f"It's raining on the site.")
        # Perform some operations for rainy weather condition
    return weather_condition


if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node('weather_data')
        pub=rospy.Publisher('weather_info',String,queue_size=10)
        rate=rospy.rate(10)
        weather_condition=acquire_weather_data
        for i in range(2):
            pub.publish(weather_condition)
        
        # Checking every 45 minutes
        rospy.sleep(2700)
    rospy.spin()
