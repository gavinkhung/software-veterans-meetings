import React, { useState, useEffect } from 'react';

import { Button, Container, Header, Input, List } from 'semantic-ui-react';
import 'semantic-ui-css/semantic.min.css'
import './App.css';

// https://react.semantic-ui.com/

const App = () => {
  const [num, setNum] = useState(1);
  const [api, setApi] = useState("");
  const [list, setList] = useState([]);

  useEffect(() => {
    console.log("Rendered for the first time");
  }, []);

  useEffect(() => {
    console.log("num changed");
    fetchAPI()
  }, [num]);

  const addNum = () => {
    if(num<60) setNum(num+1);
  }

  const subtractNum = () => {
    if(num>1) setNum(num-1);
  }

  const fetchAPI = async () => {
    const response = await fetch("https://swapi.dev/api/planets/"+num);
    const data = await response.json();
    setApi(JSON.stringify(data, undefined, 2));
  }

  const handleSubmit = (event) => {
    event.preventDefault();
    const message = event.target.elements["message"].value;
    setList([...list, message]);
  }

  return (
    <Container>
      <Header as="h1">My first React Website</Header>
      <div>
        <p>{num}</p>
        <Button onClick={addNum}>Add 1</Button>
        <Button onClick={subtractNum}>Subtract 1</Button>
      </div>
      <div>
        <pre>
          <code>{api}</code>
        </pre>
      </div>

      <div>
        <form onSubmit={handleSubmit}>
          <Input name="message" type="text"/>
          <Button>Submit</Button>
        </form>
        <List divided selection>
          {list.map(value => (
            <List.Item>{value}</List.Item>
          ))}
        </List>
      </div>
    </Container>
  );
}

export default App;