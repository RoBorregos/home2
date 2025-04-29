/// Robot service command functionality

/// Robot command action enum
// enum CommandAction {
//   Go
//   FindObject
//   Pick
//   FindPerson
//   FindPersonByName
//   Count
//   ContextualSay
//   FindPersonInfo
//   VisualInfo
//   AskAnswerQuestion
//   FollowPersonUntil
//   GuideTo
//   Give
//   Say
//   Place
// }

type CommandAction = "go" | "find_object" | "pick" | "find_person" | "find_person_by_name" | "count" | "contextual_say" | "find_person_info" | "visual_info" | "ask_answer_question" | "follow_person_until" | "guide_to" | "give" | "place"

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
Respond in the following format:
<think>
...
</think>
<answer>
...
</answer>

For the answer it should be a json that can be parsed as a CommandListShape.

class CommandShape(BaseModel):
    action: str
    characteristic: str = ""
    complement: str = ""

class CommandListShape(BaseModel):
    commands: List[CommandShape]

These are the actions that you can use:

1. go (complement = location_to_go, characteristic = "")
2. pick(complement = "object to pick", characteristic = "")
3. place(complement = "", characteristic = "")
4. contextual_say(complement = complete_command, characteristic = previous_command | information_to_answer)
5. ask_answer_question(complement = "", characteristic = "")
6. visual_info(complement = ("biggest" | "largest" | "smallest" | "heaviest" | "lightest" | "thinnest"), characteristic = object_category | "object")
7. give(complement = "", characteristic = "")
8. follow_person_until(complement = room | location | 'canceled', characteristic = name | "")
9. guide_to(complement = "person" | name, characteristic = loc_room)
10. find_person_info(complement = "name" | "pose" | "gesture", characteristic = "")
11. find_object(complement = placement_location | room, characteristic = object_to_find)
12. count(complement = placement_location | room, characteristic = object | "person with " + cloth + color | "person with " + posture | "person with " + gesture)
13. find_person(complement = gesture | posture | cloth + color | "", characteristic = "clothes" | "gesture" | "posture" | "")
14. find_person_by_name(complement = name, characteristic = "")

For the contextual_say, the complement is used as a prompt for our LLM, and the characteristic specifies the context that should be fetched and passed to that LLM.
For find_person and find_person_info, the robot approaches the specified person, so you do not need to specify that extra action.

Do not assume that a complement or a characteristic can be omitted unless it is specified on the action by having it as "". It is better to have unnecessary extra information than missing needed information.

Another example, for the prompt "locate a standing person in the bedroom and follow them", your answer should be:

<think>
Okay, let me try to figure out how to approach this command. The user wants the robot to locate a standing person in the bedroom and then follow them.

First, the robot needs to go to the bedroom. So the first action is 'go' with complement 'bedroom'. Then, in the bedroom, the robot needs to find a person who is standing. The 'find_person' action is used here. The complement should be the posture, which is 'standing', and the characteristic would be 'posture' since that's the attribute we're searching by. Wait, looking back at the available actions, find_person has complement as the gesture, posture, cloth color, etc., and characteristic specifies "clothes", "gesture", "posture", etc. So maybe the complement is the value (like 'standing') and the characteristic is the type, which is 'posture'.

So the command would be action: find_person, complement: 'standing', characteristic: 'posture'. That makes sense because the robot is looking for a person with a specific posture.

Once the person is found, the robot needs to follow them. The 'follow_person_until' action is used here. The complement can be left as 'canceled' because the follow continues until stopped, and the characteristic would be empty unless we know the person's name. Since the user didn't mention a name, the characteristic is empty. Wait, no. The 'follow_person_until' action's complement can be a room, location, or 'canceled'. But here, we just want to follow the person regardless of where they go, until the command is canceled. So the complement would be 'canceled' to indicate that the following continues until instructed otherwise. The characteristic is the person's name, but since we don't have a name, it's left empty. So the action is follow_person_until with complement 'canceled' and characteristic empty.

Putting it all together: go to bedroom, find the person with posture standing, then follow them until canceled. The commands would be:

1. go to bedroom

2. find_person (complement: standing, characteristic: posture)

3. follow_person_until (complement: canceled, characteristic: "")
</think>
<answer> 
{"commands":[{"action":"go","complement":"bedroom","characteristic":""},{"action":"find_person","complement":"standing","characteristic":"posture"},{"action":"follow_person_until","complement":"canceled","characteristic":""}]}
</answer>

Another example, for the prompt "tell me how many dishes there are on the desk", your final answer should be this:

{"commands":[{"action":"go","complement":"desk","characteristic":""},{"action":"count","complement":"desk","characteristic":"dishes"},{"action":"go","complement":"start_location","characteristic":""},{"action":"contextual_say","complement":"tell me how many dishes there are on the desk","characteristic":"count"}]}
"#
/// Function to generate robot commands from a natural language prompt
function GenerateCommandList(request: string) -> CommandListLLM {
  client R1
  prompt #"
    {{ _.role("user") }}
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