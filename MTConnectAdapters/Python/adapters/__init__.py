# import local environment variables defined by user
from dotenv import load_dotenv
load_dotenv('.env')

from adapters.mtcadapter import MTCAdapter
from adapters.mtcadapter import AgentRequestHandler
