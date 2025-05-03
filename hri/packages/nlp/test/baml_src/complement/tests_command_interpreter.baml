/// Tests for command interpreter functionality

/// Test for navigating to a location and retrieving an object
test GoToLocTest {
  functions [GenerateCommandList]
  args {
    request "go to the living room then locate a knife and get it and bring it to Angel in the bathroom"
  }
  @@assert({{this.commands|length == 5}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "living room"}})
  @@assert({{this.commands[1].action == "PickObject" and this.commands[1].complement == "knife"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "bathroom"}})
  @@assert({{this.commands[3].action == "FindPerson" and this.commands[3].complement == "Angel"}})
  @@assert({{this.commands[4].action == "GiveObject" and this.commands[4].complement is none}})
}

/// Test for taking an object from a placement and delivering it
test TakeObjFromPlcmtTest {
  functions [GenerateCommandList]
  args {
    request "take a pear from the sofa and deliver it to Charlie in the living room"
  }
  @@assert({{this.commands|length == 5}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "sofa"}})
  @@assert({{this.commands[1].action == "PickObject" and this.commands[1].complement == "pear"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "living room"}})
  @@assert({{this.commands[3].action == "FindPerson" and this.commands[3].complement == "Charlie"}})
  @@assert({{this.commands[4].action == "GiveObject" and this.commands[4].complement is none}})
}

/// Test for finding a person with specific gestures in a room
test FindPrsInRoomTest {
  functions [GenerateCommandList]
  args {
    request "locate a person pointing to the right in the bedroom and follow them to the dishwasher"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "bedroom"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == "pointing to the right"}})
  @@assert({{this.commands[2].action == "FollowPersonUntil" and this.commands[2].complement == "dishwasher"}})
}

/// Test for finding an object in a room and placing it elsewhere
test FindObjInRoomTest {
  functions [GenerateCommandList]
  args {
    request "look for a toy in the office then get it and put it on the kitchen table"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "office"}})
  @@assert({{this.commands[1].action == "PickObject" and this.commands[1].complement == "toy"}})
  @@assert({{this.commands[2].action == "GoTo" and "kitchen" in this.commands[2].complement}})
  @@assert({{this.commands[3].action == "PlaceObject" and this.commands[3].complement is none}})
}

/// Test for meeting a person at a location and answering questions
test MeetPrsAtBeacTest {
  functions [GenerateCommandList]
  args {
    request "meet Angel in the bathroom and answer a quiz"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "bathroom"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == "Angel"}})
  @@assert({{this.commands[2].action == "AnswerQuestion" and this.commands[2].complement is none}})
}

/// Test for counting objects on a placement
test CountObjOnPlcmtTest {
  functions [GenerateCommandList]
  args {
    request "tell me how many foods there are on the bookshelf"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "bookshelf"}})
  @@assert({{this.commands[1].action == "Count" and this.commands[1].complement == "foods"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "start_location"}})
  @@assert({{this.commands[3].action == "SayWithContext"}})
  @@assert({{ "how many foods there are on the bookshelf" in this.commands[3].complement }})
}

/// Test for counting persons in a room with specific characteristics
test CountPrsInRoomTest {
  functions [GenerateCommandList]
  args {
    request "tell me how many persons pointing to the left are in the kitchen"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "kitchen"}})
  @@assert({{this.commands[1].action == "Count" and this.commands[1].complement == "persons pointing to the left"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "start_location"}})
  @@assert({{this.commands[3].action == "SayWithContext"}})
  @@assert({{ "persons pointing to the left are in the kitchen" in this.commands[3].complement }})
}

/// Test for retrieving information about a person at a location
test TellPrsInfoInLocTest {
  functions [GenerateCommandList]
  args {
    request "tell me the name of the person at the lamp"
  }
  @@assert({{this.commands|length == 5}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "lamp"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == ""}})
  @@assert({{this.commands[2].action == "GetPersonInfo" and this.commands[2].complement == "name"}})
  @@assert({{this.commands[3].action == "GoTo" and this.commands[3].complement == "start_location"}})
  @@assert({{this.commands[4].action == "SayWithContext"}})
  @@assert({{ "name of the person at the lamp" in this.commands[4].complement }})
}

/// Test for identifying object properties on a placement
test TellObjPropOnPlcmtTest {
  functions [GenerateCommandList]
  args {
    request "tell me what is the thinnest object on the kitchen table"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "kitchen table"}})
  @@assert({{this.commands[1].action == "GetVisualInfo" and this.commands[1].complement == "thinnest object"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "start_location"}})
  @@assert({{this.commands[3].action == "SayWithContext"}})
  @@assert({{ "thinnest object on the kitchen table" in this.commands[3].complement }})
}

/// Test for conveying information to a person with specific gestures
test TalkInfoToGestPrsInRoomTest {
  functions [GenerateCommandList]
  args {
    request "say your team's affiliation to the waving person in the bathroom"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "bathroom"}})
  @@assert({{this.commands[1].action == "FindPerson" and "waving" in this.commands[1].complement}})
  @@assert({{this.commands[2].action == "SayWithContext"}})
  @@assert({{ "team's affiliation" in this.commands[2].complement }})
}

/// Test for answering questions from a gesturing person
test AnswerToGestPrsInRoomTest {
  functions [GenerateCommandList]
  args {
    request "answer the quiz of the person raising their left arm in the bedroom"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "bedroom"}})
  @@assert({{this.commands[1].action == "FindPerson" and "raising their left arm" in this.commands[1].complement}})
  @@assert({{this.commands[2].action == "AnswerQuestion" and this.commands[2].complement is none}})
}

/// Test for following a named person from one location to another
test FollowNameFromBeacToRoomTest {
  functions [GenerateCommandList]
  args {
    request "follow Jules from the kitchen table to the bathroom"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "kitchen table"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == "Jules"}})
  @@assert({{this.commands[2].action == "FollowPersonUntil" and this.commands[2].complement == "bathroom"}})
}

/// Test for guiding a named person between locations
test GuideNameFromBeacToBeacTest {
  functions [GenerateCommandList]
  args {
    request "lead Robin from the side tables to the office"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "side tables"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == "Robin"}})
  @@assert({{this.commands[2].action == "GuideTo" and this.commands[2].complement == "office"}})
}

/// Test for guiding a person based on posture between locations
test GuidePrsFromBeacToBeacTest {
  functions [GenerateCommandList]
  args {
    request "take the lying person from the kitchen table to the desk lamp"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "kitchen table"}})
  @@assert({{this.commands[1].action == "FindPerson" and "lying" in this.commands[1].complement}})
  @@assert({{this.commands[2].action == "GuideTo" and this.commands[2].complement == "desk lamp"}})
}

/// Test for guiding a person based on clothing between locations
test GuideClothPrsFromBeacToBeacTest {
  functions [GenerateCommandList]
  args {
    request "take the person wearing a white t shirt from the entrance to the armchair"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "entrance"}})
  @@assert({{this.commands[1].action == "FindPerson" and "white t shirt" in this.commands[1].complement}})
  @@assert({{this.commands[2].action == "GuideTo" and this.commands[2].complement == "armchair"}})
}

/// Test for bringing an object from a placement to the user
test BringMeObjFromPlcmtTest {
  functions [GenerateCommandList]
  args {
    request "give me a pear from the cabinet"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "cabinet"}})
  @@assert({{this.commands[1].action == "PickObject" and this.commands[1].complement == "pear"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "start_location"}})
  @@assert({{this.commands[3].action == "GiveObject" and this.commands[3].complement is none}})
}

/// Test for identifying properties of a specific category on a placement
test TellCatPropOnPlcmtTest {
  functions [GenerateCommandList]
  args {
    request "tell me what is the thinnest snack on the storage rack"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "storage rack"}})
  @@assert({{this.commands[1].action == "GetVisualInfo" and this.commands[1].complement == "thinnest snack"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "start_location"}})
  @@assert({{this.commands[3].action == "SayWithContext"}})
  @@assert({{ "thinnest snack on the storage rack" in this.commands[3].complement }})
}

/// Test for greeting a person by clothing description and answering a question
test GreetClothDscInRmTest {
  functions [GenerateCommandList]
  args {
    request "greet the person wearing a white coat in the kitchen and answer a question"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "kitchen"}})
  @@assert({{this.commands[1].action == "FindPerson" and "white coat" in this.commands[1].complement}})
  @@assert({{this.commands[3].action == "AnswerQuestion" and this.commands[3].complement is none}})
}

/// Test for greeting a named person and following them
test GreetNameInRmTest {
  functions [GenerateCommandList]
  args {
    request "say hello to Axel in the office and follow them"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "office"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == "Axel"}})
  @@assert({{this.commands[2].action == "FollowPersonUntil" and this.commands[2].complement == "cancelled"}})
}

/// Test for meeting a person at one location and finding them in another
test MeetNameAtLocThenFindInRmTest {
  functions [GenerateCommandList]
  args {
    request "meet Axel at the coatrack then locate them in the bathroom"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "coatrack"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == "Axel"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "bathroom"}})
  @@assert({{this.commands[3].action == "FindPerson" and this.commands[3].complement == "Axel"}})
}

/// Test for counting people with specific clothing in a room
test CountClothPrsInRoomTest {
  functions [GenerateCommandList]
  args {
    request "tell me how many people in the living room are wearing white shirts"
  }
  @@assert({{this.commands|length == 4}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "living room"}})
  @@assert({{this.commands[1].action == "Count" and this.commands[1].complement == "people wearing white shirts"}})
  @@assert({{this.commands[2].action == "GoTo" and this.commands[2].complement == "start_location"}})
  @@assert({{this.commands[3].action == "SayWithContext"}})
  @@assert({{ "how many people in the living room are wearing white shirts" in this.commands[3].complement }})
}

/// Test for conveying information about a person to another person
test TellPrsInfoAtLocToPrsAtLocTest {
  functions [GenerateCommandList]
  args {
    request "tell the pose of the person at the chairs to the person at the kitchen table"
  }
  @@assert({{this.commands|length == 6}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "chairs"}})
  @@assert({{this.commands[1].action == "FindPerson" and this.commands[1].complement == ""}})
  @@assert({{this.commands[2].action == "GetPersonInfo" and this.commands[2].complement == "pose"}})
  @@assert({{this.commands[3].action == "GoTo" and this.commands[3].complement == "kitchen table"}})
  @@assert({{this.commands[4].action == "FindPerson" and this.commands[4].complement == ""}})
  @@assert({{this.commands[5].action == "SayWithContext"}})
  @@assert({{ "pose" in this.commands[5].complement }})
}

/// Test for following a person based on posture at a location
test FollowPrsAtLocTest {
  functions [GenerateCommandList]
  args {
    request "follow the sitting person in the kitchen"
  }
  @@assert({{this.commands|length == 3}})
  @@assert({{this.commands[0].action == "GoTo" and this.commands[0].complement == "kitchen"}})
  @@assert({{this.commands[1].action == "FindPerson" and "sitting" in this.commands[1].complement}})
  @@assert({{this.commands[2].action == "FollowPersonUntil" and this.commands[2].complement == "cancelled"}})
}
