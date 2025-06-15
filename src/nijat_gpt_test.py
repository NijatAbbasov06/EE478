import time
from openai import OpenAI
import os
import ast, re
import rospy
from std_msgs.msg import String




organization_key = ""
api_key = ""

gpt_model = "gpt-4o"





def callback(msg):
    latest_qr_message = msg.data
    GPTTaskPlan(latest_qr_message)
    return 




def GPTTaskPlan(latest_qr_message):
    prompt = latest_qr_message
 


    
    client = OpenAI(
        api_key=os.environ.get("OPENAI_API_KEY", api_key),
        organization = organization_key
    ) 
    max_tokens = 1

    
    response = client.chat.completions.create(
        model=gpt_model,
        temperature=0,
        messages=[
            {
                "role": "system",
                "content": "Forget the previous conversation. I will ask you questions of the following type: LEFT: Tag ID: 8\nRIGHT: Tag ID: 9\nQ: Which number is not divisible by 3? Only answer with LEFT or RIGHT",
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                ],
            }
        ],
        max_tokens=max_tokens,

    )
    result = response.choices[0].message.content
    print(result)





def ros_node():
    rospy.init_node('GPTTaskPlan');
    rospy.Subscriber('/qr_codes', String, callback);
    rospy.spin();

def initialize_gpt():
    GPTTaskPlan("I will ask you questions of the following type: LEFT: Tag ID: 8\nRIGHT: Tag ID: 9\nQ: Which number is not divisible by 3? Only answer with LEFT or RIGHT")

initialize_gpt()

ros_node()

