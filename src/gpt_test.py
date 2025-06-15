import time
from openai import OpenAI
import os
import ast, re

organization_key = ""
api_key = ""

gpt_model = "gpt-4o"

def GPTTaskPlan():
    prompt = "LEFT: Tag ID: 8\nRIGHT: Tag ID: 9\nQ: Which number is not divisible by 3?"

    client = OpenAI(
        api_key=os.environ.get("OPENAI_API_KEY", api_key),
        organization = organization_key
    ) 
    max_tokens = 300
    
    response = client.chat.completions.create(
        model=gpt_model,
        messages=[
            {
                "role": "system",
                "content": "Forget the previous conversation.",
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

GPTTaskPlan()