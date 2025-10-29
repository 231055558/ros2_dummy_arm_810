from zai import ZhipuAiClient
client = ZhipuAiClient(api_key="87abced24cc44e58b80e18e09fd46c86.kM62rHnlo7ve7VHE")  # 请填写您自己的 APIKey
response = client.chat.completions.create(
    model="glm-4.1v-thinking-flash",  # 请填写您要调用的模型名称
    messages=[
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "Tell me the position of the couple in the picture. The short-haired guy is wearing a pink top and blue shorts, and the girl is in a cyan dress. Answer in [x1,y1,x2,y2] format."},
                {"type": "image_url", "image_url": {"url": "https://cdn.bigmodel.cn/markdown/1754968795362glm-4.5v-12.png?attname=glm-4.5v-12.png"}}
            ]
        }
    ]
)
print(response.choices[0].message.content)