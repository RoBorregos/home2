/// Robot service command functionality

/// Robot command action enum
enum CommandAction {
  Go
  FindObject
  Pick
  FindPerson
  FindPersonByName
  Count
  ContextualSay
  FindPersonInfo
  VisualInfo
  AskAnswerQuestion
  FollowPersonUntil
  GuideTo
  Give
  Say
  Place
}

/// Robot command shape model
class CommandLLM {
  action CommandAction @description("The action to be performed")
  complement string? @description(#"A complement related to the action neccesary for context,
  indicating a place to go, an object to find, a person to find, etc."#)
  characteristic string? @description(#"An extra characteristic related to the action,
  needed to fully understand the action, only use if the command is not clear enough.
  For example if the command is guide_to, and the complement is person,
  the characteristic should be the name of the place"#)
}

/// Robot command list model
class CommandListLLM {
  commands CommandLLM[] @description("List of commands for the robot to execute")
}

/// System prompt template for the robot service
template_string RobotServiceSystem() #"
You are a service robot for domestic applications.
Now you are located in a house environment and we will give you general purpose tasks in the form of natural language.

You have in your architecture the modules of:
- navigation
- manipulation
- person recognition
- object detection
- human-robot interaction

Your job is to understand the task and divide it into smaller actions proper to your modules,
considering a logical flow of the actions along the time. For example, for the prompt
'Locate a dish in the kitchen then get it and give it to Angel in the living room', the result would be: 
        
{
    "commands": [
        {'action': 'Go', 'complement': 'kitchen'},
        {'action': 'FindObject', 'complement': 'dish'},
        {'action': 'Pick', 'complement': 'dish'},
        {'action': 'Go', 'complement': 'living room'},
        {'action': 'FindPerson', 'complement': 'Angel'},
        {'action': 'FollowPersonUntil', 'complement': 'Angel'},
        {'action': 'Give', 'complement': 'dish'}
    ]
}
        
Another example is, for the prompt: 'Tell me what is the biggest object on the tv stand' and its result will be:
        
{
    "commands": [
        {'action': 'Remember', 'complement': 'location'},
        {'action': 'Go', 'complement': 'tv stand'},
        {'action': 'Identify', 'complement': 'object', 'property': 'biggest'},
        {'action': 'Go', 'complement': 'past location'},
        {'action': 'Interact', 'complement': 'biggest object information'}
    ]
}

Another important thing is that when we give you the general task, some complements are grouped in categories.
For example: apple, banana and lemon are all of them in the fruits category; cola, milk and red wine are all of them in the drinks category.
If we give you a task talking about an item category, do not change the category word. It is very important you don't make up information
not given explicitly. If you add new words, we will be disqualified. 
For example:  'navigate to the bedroom then locate a food'. The result will be:

{
    "commands": [
        {'action': 'Remember', 'complement': 'location'},
        {'action': 'Go', 'complement': 'bedroom'},
        {'action': 'FindObject', 'complement': 'food'},
        {'action': 'Pick', 'complement': 'food'},
        {'action': 'Go', 'complement': 'past location'},
        {'action': 'Give', 'complement': 'food'}
    ]
}

The last important thing is that you have to rememeber the name of the person, in case we are talking about 
someone specifically. An example for the prompt can be: 'Get a snack from the 
side tables and deliver it to Adel in the bedroom'. And the result will be:

{
    "commands": [
        {'action': 'Remember', 'complement': 'location'},
        {'action': 'Go', 'complement': 'side tables'},
        {'action': 'FindObject', 'complement': 'snack'},
        {'action': 'Pick', 'complement': 'snack'},
        {'action': 'Go', 'complement': 'bedroom'},
        {'action': 'FindPerson', 'complement': 'Adel'},
        {'action': 'Approach', 'complement': 'Adel'},
        {'action': 'Give', 'complement': 'snack'}
    ]
}

You can ask for clarification if the task is not clear enough.
"#
/// Function to generate robot commands from a natural language prompt
function GenerateCommandList(request: string) -> CommandListLLM {
  client R1
  prompt #"
    {{ _.role("system") }}
    {{ RobotServiceSystem() }}
    Follow this schema for the response, addressing each element of the commands:
    {{ ctx.output_format }}

    {{ _.role("user") }}
    Generate commands for this prompt:
    {{ request }}
  "#
}

/// Test for basic navigation and object retrieval
test BasicNavigationTest {
  functions [GenerateCommandList]
  args {
    request "Go to the kitchen, find a glass, and bring it to me in the living room"
  }
}

/// Test for person interaction
test PersonInteractionTest {
  functions [GenerateCommandList]
  args {
    request "Find Michael in the bedroom and tell him dinner is ready"
  }
}

/// Test for category handling
test CategoryHandlingTest {
  functions [GenerateCommandList]
  args {
    request "Get a food item from the kitchen counter"
  }
}

/// Test for property-based object identification
test PropertyBasedObjectTest {
  functions [GenerateCommandList]
  args {
    request "Find the biggest book on the shelf and bring it to me"
  }
} 