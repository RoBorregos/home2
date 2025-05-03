/// Robot service command functionality

/// Robot command action enum
enum CommandAction {
  GoTo @description("Go to a specific location which can be a room, furniture from a room, start_location, etc., complement is the location or 'start_location'")
  PickObject @description("Pick a specific object, complement is the name of the object")
  FindPerson @description("Find a person which can be identified by the complement, the complement can be a name or a feature/pose (pointing to the left, standing, blue shirt, etc.), if it's not clear a complement leave it empty. The command internally will greet the person, there's no need to add a greeting afterwards.")
  Count @description("Count the number of objects/persons, complement is the name of the object/person, can be a category or plural, and can be persons doing an action. Use if the user asks 'how many'")
  SayWithContext @description("Use to say a message to the user (just the intend without a previous verb) , complement is just the intent or purpose of the message, the system will receive previous interactions of the robot to define the actual message to be delivered")
  GetPersonInfo @description("Get information about a person, complement is the information to be retrieved like name, clothing, age, etc. Can only be used after a person is found by the FindPerson action")
  GetVisualInfo @description("Get information about the visualized environment, complement is the information to be retrieved like color of the object, shape, smallest object, thinnest snack,etc.")
  AnswerQuestion @description("Answer a question to be asked by a person, DOES NOT have a complement, just trigger the action, all the conversation will be handled by the system")
  FollowPersonUntil @description("Follow a person until a specific destination or cancelled, complement is the name of the location or 'cancelled'. Just use after a person is found by the FindPerson action")
  GuideTo @description("Guide a person to a specific destination, complement is the name of the location")
  GiveObject @description("Give the holding object to a person, it DOES NOT have a complement, since it is the current holding object already stored in the system's memory")
  PlaceObject @description("Place the current holding object on a specific location, it DOES NOT have a complement, since it is the current holding object already stored in the system's memory")
}

//type CommandAction = "go" | "find_object" | "pick" | "find_person" | "find_person_by_name" | "count" | "contextual_say" | "find_person_info" | "visual_info" | "ask_answer_question" | "follow_person_until" | "guide_to" | "give" | "place"

/// Robot command shape model
class CommandLLM {
  action CommandAction @description("The action to be performed")
  complement string? @description(#"A complement related to the action neccesary for context,
  indicating a place to go, an object to find, a person to find, etc. Not needed by all the actions,
  check the description of the action to see if it's needed"#)
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
         {'action': 'GoTo', 'complement': 'kitchen'},
         {'action': 'PickObject', 'complement': 'dish'},
         {'action': 'GoTo', 'complement': 'living room'},
         {'action': 'FindPerson', 'complement': 'Angel'},
         {'action': 'GiveObject'}
     ]
 }
 
 Another important thing is that when we give you the general task, some complements are grouped in categories.
 For example: apple, banana and lemon are all of them in the fruits category; cola, milk and red wine are all of them in the drinks category.
 If we give you a task talking about an item category, do not change the category word. It is very important you don't make up information
 not given explicitly. If you add new words, we will be disqualified. 
 For example:  'navigate to the bedroom then locate a food'.
 
 Another important thing is that you have to rememeber the name of the person, in case we are talking about 
 someone specifically. An example for the prompt can be: 'Get a snack from the 
 side tables and deliver it to Adel in the bedroom'.

 The system will handle the instructions and collect the response for each command.
 You can set the commands to use information from previous commands and context, for example:
 'tell me how many foods there are on the bookshelf'
 {
  "commands": [
    {"action": "GoTo", "complement": "bookshelf"},
    {"action": "Count", "complement": "foods"},
    {"action": "GoTo", "complement": "start_location"},
    {"action": "SayWithContext", "complement": "how many foods there are on the bookshelf"}
  ]
 }
 Note from this example that if a user request something to be told or given back to their,
 you should first 'GoTo' the start_location and then perform the action.

You can ask for clarification if the task is not clear enough.

"#
/// Function to generate robot commands from a natural language prompt
function GenerateCommandList(request: string) -> CommandListLLM {
  client API_QWEN3_14B
  prompt #"
    {{ _.role("user") }}
    {{ RobotServiceSystem() }}
    Follow this schema for the response, addressing each element of the commands,
    return in JSON format:
    {{ ctx.output_format }}

    {{ _.role("user") }}
    Generate commands for this prompt in the specified format::
    {{ request }}
  "#
}
