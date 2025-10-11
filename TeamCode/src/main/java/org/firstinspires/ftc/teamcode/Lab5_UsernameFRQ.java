package org.firstinspires.ftc.teamcode;

public class Lab5_UsernameFRQ {

    public static void main(String[] args) {

        String first = "Alex";
        String last = "Kim";

        // Step 1: Build a base username.
        // Take the first 2 letters of first and last 3 letters of last.
        // Use substring with indexes and length() where needed.
        String first2 = first.substring(0,2);
        String base = first2 + last;


        // Step 2: Lowercase version.
        // String lowerBase = base.toLowerCase();
        String lowerBase = base.toLowerCase();

        // Step 3: Add an identifier number.
        // Take the length of first + length of last, turn it into a string,
        // and concatenate onto lowerBase.
        String idenString = first + last;
        int iden = idenString.length();
        String finaluser = lowerBase + iden;

        // Step 4: Print final username in the format:
        // USERNAME: <string>
        System.out.println("USERNAME:" + finaluser);
    }
}

